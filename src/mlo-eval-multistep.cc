/* ================================================================
 * mlo-eval-multistep.cc  --  Wi-Fi 7 MLO Scheduler: Multi-Flow Scenario
 *
 * Replicates the MCAB paper's dynamic scenario:
 *   4 flows start at t=2, 4, 6, 8 s (each 80 Mbps).
 *   Total offered load ramps: 80 -> 160 -> 240 -> 320 Mbps.
 *
 * Purpose: observe how each scheduler reacts to load changes over time.
 * Key output: CO time-series (_sched.csv) showing link balance dynamics.
 *
 * Schedulers:
 *   sched=0  MLO-STR    (Greedy EDCA)
 *   sched=1  Adaptive OA (proportional CO-equalizing, K=0.2)
 *   sched=2  MCAB        (direct CO-ratio split)
 *
 * Topology: 1 AP -> 1 STA, downlink UDP, STR mode
 *   Link0: 5 GHz, 40 MHz, EhtMcs5  (~95 Mbps MAC)
 *   Link1: 6 GHz, 80 MHz, EhtMcs11 (~400 Mbps MAC)
 *
 * Build:
 *   ./ns3 build mlo-eval-multistep
 * Run:
 *   ./ns3 run "mlo-eval-multistep --sched=1 --seed=1"
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
NS_LOG_COMPONENT_DEFINE("MloEvalMultistep");

/* ── Global state ─────────────────────────────────────────────────────────── */
static std::vector<double> g_allSamples;
static uint64_t            g_rxBytes{0};
static uint64_t            g_rxPrimary{0};
static double              g_coSum0{0}, g_coSum1{0};
static uint32_t            g_coCount{0};
static std::ofstream       g_schedCsv;
static double              g_adaptiveP{0.808};

/* ── AdjTidClient ─────────────────────────────────────────────────────────── */
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
                .AddAttribute("PacketSize", "Payload bytes",
                              UintegerValue(1000),
                              MakeUintegerAccessor(&AdjTidClient::m_pktSize),
                              MakeUintegerChecker<uint32_t>())
                .AddAttribute("Interval", "Inter-packet gap",
                              TimeValue(MicroSeconds(500)),
                              MakeTimeAccessor(&AdjTidClient::m_interval),
                              MakeTimeChecker())
                .AddAttribute("PTid1", "Prob of TID3 (->Link1)",
                              DoubleValue(0.5),
                              MakeDoubleAccessor(&AdjTidClient::m_pTid1),
                              MakeDoubleChecker<double>(0.0, 1.0))
                .AddAttribute("TagPackets", "Attach TimestampTag",
                              BooleanValue(true),
                              MakeBooleanAccessor(&AdjTidClient::m_tag),
                              MakeBooleanChecker());
        return tid;
    }
    AdjTidClient() : m_rng(CreateObject<UniformRandomVariable>()) {}
    ~AdjTidClient() override = default;
    void SetRemote(const PacketSocketAddress& a) { m_peer = a; }
    void SetPTid1(double p) { m_pTid1 = std::clamp(p, 0.0, 1.0); }

  private:
    void StartApplication() override
    {
        if (!m_socket)
        {
            m_socket = Socket::CreateSocket(GetNode(),
                                            PacketSocketFactory::GetTypeId());
            m_socket->Bind();
            m_socket->Connect(m_peer);
        }
        SendPacket();
    }
    void StopApplication() override
    {
        if (m_evt.IsPending()) m_evt.Cancel();
        if (m_socket) { m_socket->Close(); m_socket = nullptr; }
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

/* ── Helpers ──────────────────────────────────────────────────────────────── */
static void
ConfigureTidToLinkMapping(NetDeviceContainer devs, const std::string& mapping)
{
    for (uint32_t i = 0; i < devs.GetN(); ++i)
    {
        auto wd = DynamicCast<WifiNetDevice>(devs.Get(i));
        if (!wd) continue;
        auto eht = wd->GetMac()->GetEhtConfiguration();
        if (!eht) continue;
        eht->SetAttribute("TidToLinkMappingNegSupport",
                          EnumValue(WifiTidToLinkMappingNegSupport::ANY_LINK_SET));
        eht->SetAttribute("TidToLinkMappingUl", StringValue(mapping));
        eht->SetAttribute("TidToLinkMappingDl", StringValue(mapping));
    }
}

static double
ComputeChannelOccupancy(WifiCoTraceHelper& h, uint32_t nodeId, uint32_t linkId)
{
    auto& recs = h.GetDeviceRecords();
    auto  it   = std::find_if(recs.begin(), recs.end(),
                              [nodeId](const auto& r){ return r.m_nodeId == nodeId; });
    if (it == recs.end()) return 0.0;
    auto jt = it->m_linkStateDurations.find(linkId);
    if (jt == it->m_linkStateDurations.end()) return 0.0;
    double idle = 0.0, total = 0.0;
    for (auto& [state, dur] : jt->second)
    {
        if (state == WifiPhyState::IDLE) idle = dur.GetDouble();
        total += dur.GetDouble();
    }
    return (total < 1e-9) ? 0.0 : (total - idle) / total;
}

static void
RxCb(Ptr<const Packet> pkt, const Address&)
{
    g_rxBytes += pkt->GetSize();
    TimestampTag ts;
    if (pkt->PeekPacketTag(ts))
    {
        double ms = (Simulator::Now() - ts.GetTimestamp()).GetSeconds() * 1e3;
        g_allSamples.push_back(ms);
        g_rxPrimary += pkt->GetSize();
    }
}

/* ── Scheduler callback ───────────────────────────────────────────────────── */
static void
SchedulerCb(WifiCoTraceHelper&             helper,
            std::vector<Ptr<AdjTidClient>> clients,
            int                            schedMode,
            Time                           period)
{
    double co0 = ComputeChannelOccupancy(helper, 0, 0);
    double co1 = ComputeChannelOccupancy(helper, 0, 1);
    helper.Reset();

    g_coSum0 += co0;
    g_coSum1 += co1;
    ++g_coCount;

    double newP;
    if (schedMode == 1)
    {
        double error = co0 - co1;
        newP         = std::clamp(g_adaptiveP + 0.2 * error, 0.05, 0.95);
        g_adaptiveP  = newP;
    }
    else if (schedMode == 2)
    {
        double sum = co0 + co1;
        newP       = (sum > 1e-9) ? std::clamp(co0 / sum, 0.05, 0.95) : 0.5;
    }
    else
    {
        newP = 1.0;
    }

    for (auto& c : clients) c->SetPTid1(newP);

    g_schedCsv << std::fixed << std::setprecision(5)
               << Simulator::Now().GetSeconds() << ","
               << co0 << "," << co1 << "," << newP << "\n";

    Simulator::Schedule(period, &SchedulerCb, std::ref(helper), clients,
                        schedMode, period);
}

static Ptr<AdjTidClient>
InstallClient(Ptr<Node> ap, Ptr<Node> sta, double loadMbps,
              double pInit, Time start, Time stop)
{
    auto apDev  = DynamicCast<WifiNetDevice>(ap->GetDevice(0));
    auto staDev = DynamicCast<WifiNetDevice>(sta->GetDevice(0));
    PacketSocketAddress addr;
    addr.SetSingleDevice(apDev->GetIfIndex());
    addr.SetPhysicalAddress(staDev->GetAddress());
    addr.SetProtocol(1);
    const uint32_t pktSize    = 1000;
    double         intervalUs = pktSize * 8.0 / loadMbps;
    auto client = CreateObject<AdjTidClient>();
    client->SetAttribute("PacketSize", UintegerValue(pktSize));
    client->SetAttribute("Interval",   TimeValue(MicroSeconds(intervalUs)));
    client->SetAttribute("PTid1",      DoubleValue(pInit));
    client->SetAttribute("TagPackets", BooleanValue(true));
    client->SetRemote(addr);
    client->SetStartTime(start);
    client->SetStopTime(stop);
    ap->AddApplication(client);
    return client;
}

/* ── Main ─────────────────────────────────────────────────────────────────── */
int
main(int argc, char* argv[])
{
    LogComponentEnable("MloEvalMultistep", LOG_LEVEL_INFO);

    uint32_t    schedType  = 0;
    uint32_t    seed       = 1;
    double      loadPerFlow = 80.0;   /* Mbps per flow step */
    uint32_t    windowMs   = 250;
    std::string outDir     = "scratch/mlo_results_multistep";

    CommandLine cmd(__FILE__);
    cmd.AddValue("sched",       "0=STR 1=AdaptiveOA 2=MCAB",  schedType);
    cmd.AddValue("seed",        "RNG seed",                     seed);
    cmd.AddValue("loadPerFlow", "Mbps added per flow step",     loadPerFlow);
    cmd.AddValue("window",      "Control window [ms]",          windowMs);
    cmd.AddValue("outDir",      "Output directory",             outDir);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(schedType > 2, "sched must be 0, 1, or 2");

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(seed);

    /* Flow schedule: start times and cumulative loads */
    const std::vector<double> flowStarts = {2.0, 4.0, 6.0, 8.0};
    const double simTime = 13.0; /* 5 s after last flow starts */
    const Time   tStop   = Seconds(simTime);

    /* ── Nodes ── */
    NodeContainer ap, sta, all;
    ap.Create(1); sta.Create(1);
    all.Add(ap); all.Add(sta);

    /* ── Channels ── */
    auto ch5 = CreateObject<MultiModelSpectrumChannel>();
    auto ch6 = CreateObject<MultiModelSpectrumChannel>();
    SpectrumWifiPhyHelper phy(2);
    phy.AddChannel(ch5, WIFI_SPECTRUM_5_GHZ);
    phy.AddChannel(ch6, WIFI_SPECTRUM_6_GHZ);
    phy.Set(0, "ChannelSettings", StringValue("{0, 40, BAND_5GHZ, 0}"));
    phy.Set(1, "ChannelSettings", StringValue("{0, 80, BAND_6GHZ, 0}"));

    /* ── Wi-Fi 7 ── */
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211be);
    wifi.SetRemoteStationManager(uint8_t{0}, "ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("EhtMcs5"));
    wifi.SetRemoteStationManager(uint8_t{1}, "ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("EhtMcs11"));
    Ssid ssid("mlo-ms");
    WifiMacHelper mac;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevs = wifi.Install(phy, mac, ap);
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer staDevs = wifi.Install(phy, mac, sta);
    NetDeviceContainer allDevs;
    allDevs.Add(apDevs); allDevs.Add(staDevs);

    /* ── Mobility ── */
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(all);

    /* ── TID mapping ── */
    const std::string tidMapping =
        (schedType == 0) ? "0 0; 3 0,1; 1,2,4,5,6,7 1"
                         : "0 0; 1,2,3,4,5,6,7 1";
    ConfigureTidToLinkMapping(allDevs, tidMapping);

    /* ── PacketSocket ── */
    PacketSocketHelper psh;
    psh.Install(all);

    /* ── Server on STA ── */
    auto dev = DynamicCast<WifiNetDevice>(sta.Get(0)->GetDevice(0));
    PacketSocketAddress srvAddr;
    srvAddr.SetSingleDevice(dev->GetIfIndex());
    srvAddr.SetProtocol(1);
    auto srv = CreateObject<PacketSocketServer>();
    srv->SetLocal(srvAddr);
    sta.Get(0)->AddApplication(srv);
    srv->SetStartTime(Seconds(1.0));
    srv->SetStopTime(tStop);
    Config::ConnectWithoutContext(
        "/NodeList/1/ApplicationList/*/$ns3::PacketSocketServer/Rx",
        MakeCallback(&RxCb));

    /* ── Install 4 clients starting at t=2, 4, 6, 8 ── */
    double initP = (schedType == 0) ? 1.0 : 0.808;
    std::vector<Ptr<AdjTidClient>> clients;
    for (double t : flowStarts)
    {
        auto c = InstallClient(ap.Get(0), sta.Get(0),
                               loadPerFlow, initP,
                               Seconds(t), tStop);
        clients.push_back(c);
    }

    /* ── WifiCoTraceHelper ── */
    WifiCoTraceHelper coHelper;
    coHelper.Enable(ap.Get(0));

    /* ── Output file ── */
    std::ostringstream pfxSS;
    pfxSS << outDir << "/s" << schedType << "_sd" << seed;
    const std::string pfx = pfxSS.str();
    g_schedCsv.open(pfx + "_sched.csv");
    NS_ABORT_MSG_IF(!g_schedCsv.is_open(), "Cannot open " << pfx << "_sched.csv");
    g_schedCsv << "time_s,co0_ap,co1_ap,p_tid1\n";

    /* ── Schedule callbacks ── */
    const Time period{MilliSeconds(windowMs)};
    Simulator::Schedule(Seconds(2.0), &SchedulerCb,
                        std::ref(coHelper), clients,
                        static_cast<int>(schedType), period);

    /* ── Run ── */
    NS_LOG_INFO("sched=" << schedType << " seed=" << seed
                << " loadPerFlow=" << loadPerFlow << " Mbps"
                << " window=" << windowMs << "ms"
                << " sim=" << simTime << "s");

    Simulator::Stop(tStop);
    Simulator::Run();
    Simulator::Destroy();
    g_schedCsv.close();

    /* ── Summary ── */
    const std::string sumPath = outDir + "/summary.csv";
    bool   needHdr = !std::ifstream(sumPath).good();
    std::ofstream scsv(sumPath, std::ios::app);
    if (needHdr)
        scsv << "sched,seed,load_per_flow,n_pkts,avg_co0,avg_co1\n";

    double avgCo0 = g_coCount ? g_coSum0 / g_coCount : 0.0;
    double avgCo1 = g_coCount ? g_coSum1 / g_coCount : 0.0;
    scsv << schedType << "," << seed << "," << loadPerFlow << ","
         << g_allSamples.size() << ","
         << std::fixed << std::setprecision(4) << avgCo0 << "," << avgCo1 << "\n";
    scsv.close();

    std::cout << "[DONE] sched=" << schedType << " seed=" << seed
              << " | pkts=" << g_allSamples.size()
              << " | avg_co0=" << std::fixed << std::setprecision(3) << avgCo0
              << " co1=" << avgCo1 << "\n";
    return 0;
}
