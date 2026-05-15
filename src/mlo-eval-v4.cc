/* ================================================================
 * mlo-eval-v4.cc  --  Wi-Fi 7 (802.11be EHT) MLO Scheduler Evaluation
 *
 * Compares three schedulers under varying UDP load:
 *
 *   sched=0  MLO-STR (Greedy / FCFS)
 *            Default ns-3 behavior: both links independently contend
 *            via EDCA; HOL packet goes on whichever link wins first.
 *
 *   sched=1  Adaptive OA (occupancy-aware, CO-equalizing)
 *            Proportional controller every <window> ms:
 *              p <- clamp(p + K*(co0-co1), 0.05, 0.95)   K=0.2
 *            Fixed point: co0==co1 (equalization).
 *
 *   sched=2  MCAB (Multi-Link Congestion-Aware Load Balancing)
 *            Per-window direct ratio:
 *              p = clamp(co0/(co0+co1), 0.05, 0.95)
 *            Ref: Lopez-Raventos & Bellalta, IEEE WCL 2022.
 *
 * New vs v3: --window parameter (ms) sets measurement + control period.
 *            Default 250 ms; also compare 125 ms and 100 ms.
 *
 * Topology (downlink AP -> STA):
 *   AP -- Link0 (5 GHz, 40 MHz, EhtMcs5 ) --> STA  (~95 Mbps MAC)
 *   AP -- Link1 (6 GHz, 80 MHz, EhtMcs11) --> STA  (~400 Mbps MAC)
 *
 * TID->Link mapping:
 *   sched=0 (Greedy) : TID3->{Link0,Link1}; EDCA decides per-packet
 *   sched=1,2        : TID0->Link0 only, TID3->Link1 only;
 *                      SetPTid1 probability steers traffic split
 *
 * Outputs (all in outDir/):
 *   s{S}_l{L}_w{W}_sd{R}_sched.csv   -- CO + p_tid1 per window
 *   s{S}_l{L}_w{W}_sd{R}_lat.csv     -- latency stats per window
 *   s{S}_l{L}_w{W}_sd{R}_samples.csv -- per-packet latency
 *   outDir/summary.csv                -- one row per run (appended)
 *
 * Build:
 *   cd /home/mynavajha/ns-3-dev && ./ns3 build mlo-eval-v4
 *
 * Run examples:
 *   ./ns3 run "mlo-eval-v4 --sched=0 --load=310 --window=250 --seed=1"
 *   ./ns3 run "mlo-eval-v4 --sched=1 --load=310 --window=125 --seed=1"
 *   ./ns3 run "mlo-eval-v4 --sched=2 --load=310 --window=100 --seed=1"
 * ================================================================ */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/network-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/ssid.h"
#include "ns3/timestamp-tag.h"
#include "ns3/wifi-co-trace-helper.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("MloEvalV4");

/* ================================================================
 * GLOBAL STATE
 * ================================================================ */
static std::vector<double> g_latWindow;
static std::vector<double> g_allSamples;
static uint64_t            g_rxBytes{0};
static uint64_t            g_rxPrimary{0};
static uint64_t            g_prevRxBytes{0};

static double   g_coSum0{0.0};
static double   g_coSum1{0.0};
static uint32_t g_coCount{0};

static std::ofstream g_schedCsv;
static std::ofstream g_latCsv;
static std::ofstream g_sampleCsv;

/* EMA state for Adaptive OA (sched=1) only */
static double g_adaptiveP{0.808};

/* ================================================================
 * ADJUSTABLE-TID DOWNLINK CLIENT
 * ================================================================ */
class AdjTidClient : public Application
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("AdjTidClient")
                .SetParent<Application>()
                .SetGroupName("Network")
                .AddConstructor<AdjTidClient>()
                .AddAttribute("PacketSize",
                              "Payload in bytes",
                              UintegerValue(1000),
                              MakeUintegerAccessor(&AdjTidClient::m_pktSize),
                              MakeUintegerChecker<uint32_t>())
                .AddAttribute("Interval",
                              "Inter-packet gap",
                              TimeValue(MicroSeconds(500)),
                              MakeTimeAccessor(&AdjTidClient::m_interval),
                              MakeTimeChecker())
                .AddAttribute("PTid1",
                              "Prob of TID3 (->Link1)",
                              DoubleValue(0.5),
                              MakeDoubleAccessor(&AdjTidClient::m_pTid1),
                              MakeDoubleChecker<double>(0.0, 1.0))
                .AddAttribute("TagPackets",
                              "Attach TimestampTag for latency",
                              BooleanValue(true),
                              MakeBooleanAccessor(&AdjTidClient::m_tag),
                              MakeBooleanChecker());
        return tid;
    }

    AdjTidClient() : m_rng(CreateObject<UniformRandomVariable>()) {}
    ~AdjTidClient() override = default;

    void SetRemote(const PacketSocketAddress& a) { m_peer = a; }
    void SetPTid1(double p) { m_pTid1 = std::clamp(p, 0.0, 1.0); }
    void SetTag(bool t) { m_tag = t; }

  private:
    void StartApplication() override
    {
        if (!m_socket)
        {
            m_socket =
                Socket::CreateSocket(GetNode(), PacketSocketFactory::GetTypeId());
            m_socket->Bind();
            m_socket->Connect(m_peer);
        }
        SendPacket();
    }

    void StopApplication() override
    {
        if (m_evt.IsPending())
            m_evt.Cancel();
        if (m_socket)
        {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

    void SendPacket()
    {
        auto pkt = Create<Packet>(m_pktSize);

        if (m_tag)
        {
            TimestampTag ts;
            ts.SetTimestamp(Simulator::Now());
            pkt->AddPacketTag(ts);
        }

        uint8_t tid = (m_rng->GetValue() < m_pTid1) ? uint8_t{3} : uint8_t{0};
        SocketPriorityTag ptag;
        ptag.SetPriority(tid);
        pkt->ReplacePacketTag(ptag);

        m_socket->Send(pkt);
        m_evt = Simulator::Schedule(m_interval, &AdjTidClient::SendPacket, this);
    }

    Ptr<Socket>                m_socket{nullptr};
    PacketSocketAddress        m_peer;
    uint32_t                   m_pktSize{1000};
    Time                       m_interval{MicroSeconds(500)};
    double                     m_pTid1{0.5};
    bool                       m_tag{true};
    EventId                    m_evt;
    Ptr<UniformRandomVariable> m_rng;
};

NS_OBJECT_ENSURE_REGISTERED(AdjTidClient);

/* ================================================================
 * TID-TO-LINK MAPPING
 * ================================================================ */
static void
ConfigureTidToLinkMapping(NetDeviceContainer devs, const std::string& mapping)
{
    for (uint32_t i = 0; i < devs.GetN(); ++i)
    {
        auto wd = DynamicCast<WifiNetDevice>(devs.Get(i));
        if (!wd)
            continue;
        auto eht = wd->GetMac()->GetEhtConfiguration();
        if (!eht)
            continue;
        eht->SetAttribute("TidToLinkMappingNegSupport",
                          EnumValue(WifiTidToLinkMappingNegSupport::ANY_LINK_SET));
        eht->SetAttribute("TidToLinkMappingUl", StringValue(mapping));
        eht->SetAttribute("TidToLinkMappingDl", StringValue(mapping));
    }
}

/* ================================================================
 * CHANNEL OCCUPANCY HELPER
 * ================================================================ */
static double
ComputeChannelOccupancy(WifiCoTraceHelper& h, uint32_t nodeId, uint32_t linkId)
{
    auto& recs = h.GetDeviceRecords();
    auto  it   = std::find_if(recs.begin(),
                             recs.end(),
                             [nodeId](const auto& r) {
                                 return r.m_nodeId == nodeId;
                             });
    if (it == recs.end())
        return 0.0;

    auto jt = it->m_linkStateDurations.find(linkId);
    if (jt == it->m_linkStateDurations.end())
        return 0.0;

    double idle = 0.0, total = 0.0;
    for (auto& [state, dur] : jt->second)
    {
        if (state == WifiPhyState::IDLE)
            idle = dur.GetDouble();
        total += dur.GetDouble();
    }
    return (total < 1e-9) ? 0.0 : (total - idle) / total;
}

/* ================================================================
 * PACKET RECEIVE CALLBACK
 * ================================================================ */
static void
RxCb(Ptr<const Packet> pkt, const Address&)
{
    g_rxBytes += pkt->GetSize();

    TimestampTag ts;
    if (pkt->PeekPacketTag(ts))
    {
        double ms = (Simulator::Now() - ts.GetTimestamp()).GetSeconds() * 1e3;
        g_latWindow.push_back(ms);
        g_allSamples.push_back(ms);
        g_rxPrimary += pkt->GetSize();

        if (g_sampleCsv.is_open())
            g_sampleCsv << std::fixed << std::setprecision(6)
                        << Simulator::Now().GetSeconds() << "," << ms << "\n";
    }
}

/* ================================================================
 * PERIODIC LATENCY + THROUGHPUT STATS LOG
 * ================================================================ */
static void
LogStats(Time period)
{
    double bytesWin   = static_cast<double>(g_rxBytes - g_prevRxBytes);
    g_prevRxBytes     = g_rxBytes;
    double tputMbps   = bytesWin * 8.0 / period.GetSeconds() / 1e6;

    double   mean{0}, p50{0}, p90{0}, p99{0}, maxv{0};
    uint64_t n = g_latWindow.size();

    if (n > 0)
    {
        auto v = g_latWindow;
        std::sort(v.begin(), v.end());
        mean     = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        auto pct = [&](double p) -> double {
            size_t i =
                std::min(v.size() - 1,
                         static_cast<size_t>(p / 100.0 * (v.size() - 1)));
            return v[i];
        };
        p50  = pct(50);
        p90  = pct(90);
        p99  = pct(99);
        maxv = v.back();
    }
    g_latWindow.clear();

    g_latCsv << std::fixed << std::setprecision(4)
             << Simulator::Now().GetSeconds() << ","
             << n << ","
             << mean << "," << p50 << "," << p90 << "," << p99 << "," << maxv
             << "," << tputMbps << "\n";

    Simulator::Schedule(period, &LogStats, period);
}

/* ================================================================
 * SCHEDULER CALLBACK
 *
 *   sched=0 (STR Greedy)  : logs CO only; pTid1 fixed at 1.0
 *   sched=1 (Adaptive OA) : p <- clamp(p + 0.2*(co0-co1), 0.05, 0.95)
 *   sched=2 (MCAB)        : p  = clamp(co0/(co0+co1), 0.05, 0.95)
 *
 * Helper is Reset() each window so CO reflects only the latest window.
 * ================================================================ */
static void
SchedulerCb(WifiCoTraceHelper& helper,
            Ptr<AdjTidClient>  primary,
            int                schedMode,
            Time               period)
{
    double co0 = ComputeChannelOccupancy(helper, 0, 0);
    double co1 = ComputeChannelOccupancy(helper, 0, 1);
    helper.Reset();

    g_coSum0 += co0;
    g_coSum1 += co1;
    ++g_coCount;

    double newP;
    if (schedMode == 1) /* Adaptive OA: proportional CO-equalizing controller */
    {
        double error = co0 - co1;
        newP         = std::clamp(g_adaptiveP + 0.2 * error, 0.05, 0.95);
        g_adaptiveP  = newP;
        primary->SetPTid1(newP);
    }
    else if (schedMode == 2) /* MCAB: direct CO-ratio split */
    {
        double sum = co0 + co1;
        newP       = (sum > 1e-9) ? std::clamp(co0 / sum, 0.05, 0.95) : 0.5;
        primary->SetPTid1(newP);
    }
    else /* sched=0: STR Greedy; EDCA decides link */
    {
        newP = 1.0;
    }

    g_schedCsv << std::fixed << std::setprecision(5)
               << Simulator::Now().GetSeconds() << ","
               << co0 << "," << co1 << "," << newP << "\n";

    Simulator::Schedule(period,
                        &SchedulerCb,
                        std::ref(helper),
                        primary,
                        schedMode,
                        period);
}

/* ================================================================
 * SERVER + CLIENT INSTALL HELPERS
 * ================================================================ */
static void
InstallServer(Ptr<Node> sta, Time start, Time stop)
{
    auto dev = DynamicCast<WifiNetDevice>(sta->GetDevice(0));
    NS_ABORT_MSG_IF(!dev, "STA device is not WifiNetDevice");

    PacketSocketAddress addr;
    addr.SetSingleDevice(dev->GetIfIndex());
    addr.SetProtocol(1);

    auto srv = CreateObject<PacketSocketServer>();
    srv->SetLocal(addr);
    sta->AddApplication(srv);
    srv->SetStartTime(start);
    srv->SetStopTime(stop);
}

static Ptr<AdjTidClient>
InstallClient(Ptr<Node>               ap,
              Ptr<Node>               sta,
              double                  loadMbps,
              double                  pTid1Init,
              bool                    addTag,
              Time                    start,
              Time                    stop)
{
    auto apDev  = DynamicCast<WifiNetDevice>(ap->GetDevice(0));
    auto staDev = DynamicCast<WifiNetDevice>(sta->GetDevice(0));
    NS_ABORT_MSG_IF(!apDev || !staDev, "Device cast failed");

    PacketSocketAddress addr;
    addr.SetSingleDevice(apDev->GetIfIndex());
    addr.SetPhysicalAddress(staDev->GetAddress());
    addr.SetProtocol(1);

    const uint32_t pktSize    = 1000;
    double         intervalUs = pktSize * 8.0 / loadMbps;

    auto client = CreateObject<AdjTidClient>();
    client->SetAttribute("PacketSize", UintegerValue(pktSize));
    client->SetAttribute("Interval",   TimeValue(MicroSeconds(intervalUs)));
    client->SetAttribute("PTid1",      DoubleValue(pTid1Init));
    client->SetAttribute("TagPackets", BooleanValue(addTag));
    client->SetRemote(addr);
    client->SetStartTime(start);
    client->SetStopTime(stop);
    ap->AddApplication(client);

    return client;
}

/* ================================================================
 * PERCENTILE HELPER
 * ================================================================ */
struct LatStats
{
    size_t n{0};
    double mean{0}, p50{0}, p90{0}, p99{0}, p999{0}, maxv{0};
};

static LatStats
ComputeStats(std::vector<double> v)
{
    LatStats s;
    s.n = v.size();
    if (v.empty())
        return s;
    std::sort(v.begin(), v.end());
    s.mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    auto pct = [&](double p) -> double {
        size_t i =
            std::min(v.size() - 1,
                     static_cast<size_t>(p / 100.0 * (v.size() - 1)));
        return v[i];
    };
    s.p50  = pct(50);
    s.p90  = pct(90);
    s.p99  = pct(99);
    s.p999 = pct(99.9);
    s.maxv = v.back();
    return s;
}

/* ================================================================
 * MAIN
 * ================================================================ */
int
main(int argc, char* argv[])
{
    LogComponentEnable("MloEvalV4", LOG_LEVEL_INFO);

    uint32_t    schedType = 0;
    double      loadMbps  = 310.0;
    double      simTime   = 20.0;
    uint32_t    seed      = 1;
    uint32_t    windowMs  = 250;
    std::string outDir    = "scratch/mlo_results_v5/comparative";

    CommandLine cmd(__FILE__);
    cmd.AddValue("sched",   "0=STR  1=AdaptiveOA  2=MCAB", schedType);
    cmd.AddValue("load",    "UDP load [Mbps]",              loadMbps);
    cmd.AddValue("simTime", "Simulation time [s]",          simTime);
    cmd.AddValue("seed",    "RNG seed",                     seed);
    cmd.AddValue("window",  "Control/measurement window [ms]", windowMs);
    cmd.AddValue("outDir",  "Output directory",             outDir);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(schedType > 2, "sched must be 0, 1, or 2");
    NS_ABORT_MSG_IF(windowMs < 50 || windowMs > 1000,
                    "window must be in [50, 1000] ms");

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(seed);

    /* ---- Nodes ---- */
    NodeContainer ap, sta, all;
    ap.Create(1);
    sta.Create(1);
    all.Add(ap);
    all.Add(sta);

    /* ---- Spectrum channels ---- */
    auto ch5 = CreateObject<MultiModelSpectrumChannel>();
    auto ch6 = CreateObject<MultiModelSpectrumChannel>();

    SpectrumWifiPhyHelper phy(2);
    phy.AddChannel(ch5, WIFI_SPECTRUM_5_GHZ);
    phy.AddChannel(ch6, WIFI_SPECTRUM_6_GHZ);
    phy.Set(0, "ChannelSettings", StringValue("{0, 40, BAND_5GHZ, 0}"));
    phy.Set(1, "ChannelSettings", StringValue("{0, 80, BAND_6GHZ, 0}"));

    /* ---- Wi-Fi 7 with asymmetric MCS ---- */
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211be);
    wifi.SetRemoteStationManager(uint8_t{0},
                                 "ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("EhtMcs5"));
    wifi.SetRemoteStationManager(uint8_t{1},
                                 "ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("EhtMcs11"));

    Ssid ssid("mlo-v4");
    WifiMacHelper mac;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevs = wifi.Install(phy, mac, ap);

    mac.SetType("ns3::StaWifiMac",
                "Ssid",
                SsidValue(ssid),
                "ActiveProbing",
                BooleanValue(false));
    NetDeviceContainer staDevs = wifi.Install(phy, mac, sta);

    NetDeviceContainer allDevs;
    allDevs.Add(apDevs);
    allDevs.Add(staDevs);

    /* ---- Mobility ---- */
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(all);

    /* ---- TID->Link mapping ----
     * sched=0: TID3 on {Link0,Link1}; EDCA contention per packet
     * sched=1,2: TID0->Link0 only, TID3->Link1 only */
    const std::string tidMapping =
        (schedType == 0) ? "0 0; 3 0,1; 1,2,4,5,6,7 1"
                         : "0 0; 1,2,3,4,5,6,7 1";
    ConfigureTidToLinkMapping(allDevs, tidMapping);

    /* ---- PacketSocket ---- */
    PacketSocketHelper psh;
    psh.Install(all);

    Time tStart{Seconds(1.0)};
    Time tStop{Seconds(simTime)};

    InstallServer(sta.Get(0), tStart, tStop);
    Config::ConnectWithoutContext(
        "/NodeList/1/ApplicationList/*/$ns3::PacketSocketServer/Rx",
        MakeCallback(&RxCb));

    /* Initial p: capacity-weighted for OA/MCAB, 1.0 for STR */
    double initPTid1 = (schedType == 0) ? 1.0 : 0.808;
    auto   primary   = InstallClient(ap.Get(0),
                                   sta.Get(0),
                                   loadMbps,
                                   initPTid1,
                                   true,
                                   tStart,
                                   tStop);

    /* ---- WifiCoTraceHelper (AP-side only) ---- */
    WifiCoTraceHelper coHelper;
    coHelper.Enable(ap.Get(0));

    /* ---- Output files ---- */
    std::ostringstream pfxSS;
    pfxSS << outDir << "/s" << schedType
          << "_l" << static_cast<int>(loadMbps)
          << "_w" << windowMs
          << "_sd" << seed;
    const std::string pfx = pfxSS.str();

    g_schedCsv.open(pfx + "_sched.csv");
    NS_ABORT_MSG_IF(!g_schedCsv.is_open(), "Cannot open " << pfx << "_sched.csv");
    g_schedCsv << "time_s,co0_ap,co1_ap,p_tid1\n";

    g_latCsv.open(pfx + "_lat.csv");
    NS_ABORT_MSG_IF(!g_latCsv.is_open(), "Cannot open " << pfx << "_lat.csv");
    g_latCsv << "time_s,n_pkts,mean_ms,p50_ms,p90_ms,p99_ms,max_ms,tput_mbps\n";

    g_sampleCsv.open(pfx + "_samples.csv");
    NS_ABORT_MSG_IF(!g_sampleCsv.is_open(), "Cannot open " << pfx << "_samples.csv");
    g_sampleCsv << "time_s,latency_ms\n";

    /* ---- Schedule periodic callbacks ---- */
    const Time period{MilliSeconds(windowMs)};

    Simulator::Schedule(tStart,
                        &SchedulerCb,
                        std::ref(coHelper),
                        primary,
                        static_cast<int>(schedType),
                        period);

    Simulator::Schedule(tStart + period, &LogStats, period);

    /* ---- Run ---- */
    NS_LOG_INFO("Starting: sched=" << schedType
                << " load=" << loadMbps
                << " window=" << windowMs << "ms"
                << " seed=" << seed
                << " sim=" << simTime << "s");

    Simulator::Stop(tStop);
    Simulator::Run();
    Simulator::Destroy();

    g_schedCsv.close();
    g_latCsv.close();
    g_sampleCsv.close();

    /* ---- Summary row (appended to shared CSV) ---- */
    const std::string sumPath = outDir + "/summary.csv";
    bool              needHdr = !std::ifstream(sumPath).good();
    std::ofstream     scsv(sumPath, std::ios::app);
    if (needHdr)
        scsv << "sched,load_mbps,window_ms,seed,"
                "n_pkts,mean_ms,p50_ms,p90_ms,p99_ms,p999_ms,max_ms,"
                "tput_mbps,avg_co0_ap,avg_co1_ap\n";

    double elapsed  = simTime - 1.0;
    double tputMbps = g_rxPrimary * 8.0 / elapsed / 1e6;
    double avgCo0   = (g_coCount > 0) ? g_coSum0 / g_coCount : 0.0;
    double avgCo1   = (g_coCount > 0) ? g_coSum1 / g_coCount : 0.0;

    auto st = ComputeStats(g_allSamples);

    scsv << std::fixed << std::setprecision(4)
         << schedType << ","
         << loadMbps  << ","
         << windowMs  << ","
         << seed      << ","
         << st.n    << ","
         << st.mean << "," << st.p50  << "," << st.p90  << ","
         << st.p99  << "," << st.p999 << "," << st.maxv << ","
         << tputMbps << ","
         << avgCo0 << "," << avgCo1 << "\n";
    scsv.close();

    std::cout << "[DONE] sched=" << schedType
              << " load=" << loadMbps
              << " window=" << windowMs << "ms"
              << " seed=" << seed
              << " | n="   << st.n
              << " mean="  << std::fixed << std::setprecision(2) << st.mean
              << " p90="   << st.p90
              << " p99="   << st.p99  << " ms"
              << " | tput=" << std::setprecision(1) << tputMbps << " Mbps"
              << " | co0=" << std::setprecision(3) << avgCo0
              << " co1="  << avgCo1 << "\n";

    return 0;
}
