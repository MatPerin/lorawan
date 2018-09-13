/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */

#include "ns3/network-server-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/utilities.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lora-mac.h"
#include "ns3/gateway-lora-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include <algorithm>
#include <ctime>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

// Network settings
int nDevices = 1000;
int gatewayRings = 1;
int nGateways = 3 * gatewayRings * gatewayRings - 3 * gatewayRings + 1;
double radius = 6300;
double gatewayRadius = 6300 / ((gatewayRings - 1) * 2 + 1);
int appPeriodSeconds = 1000;
double simulationTime = appPeriodSeconds * 20;
std::vector<int> sfQuantity (6);

bool gwPowerAveraging = false;
bool historyAveraging = false;
uint8_t historyRange = 1;
std::string channelVariability ("2");

// Output control
  bool printEDs = true;
bool buildingsEnabled = false;

Time lastPrintTime = Seconds(0);

void
PrintPerformances (LoraHelper helper)
{
  Simulator::Schedule (Seconds (appPeriodSeconds), &PrintPerformances,
                       helper);

  if (lastPrintTime != Simulator::Now ())
    {
      helper.CountPhyPackets(lastPrintTime, Simulator::Now ());
      lastPrintTime = Simulator::Now ();
    }
}

void
PrintEndDevices (NodeContainer endDevices, NodeContainer gateways,
                 std::string filename, LoraHelper helper)
{
  const char * c = filename.c_str ();
  std::ofstream spreadingFactorFile;
  spreadingFactorFile.open (c);
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != 0);
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != 0);
      Ptr<EndDeviceLoraMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLoraMac> ();
      int sf = int(mac->GetDataRate ());
      //int tx = int(mac->GetTransmissionPower());
      //Vector pos = position->GetPosition ();
      spreadingFactorFile << sf << std::endl;
    }
  spreadingFactorFile.close ();

  int n = Simulator::Now().GetMinutes();

  std::ostringstream s;
  s << "endDevices_" << n << ".dat";
  std::string query(s.str());

  Simulator::Schedule (Seconds (appPeriodSeconds/3), &PrintEndDevices,
                       endDevices, gateways, query, helper);
}



int main (int argc, char *argv[])
{

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("gatewayRings", "Number of gateway rings to include", gatewayRings);
  cmd.AddValue ("radius", "The radius of the area to simulate", radius);
  cmd.AddValue ("gatewayRadius", "The distance between two gateways", gatewayRadius);
  cmd.AddValue ("simulationTime", "The time for which to simulate", simulationTime);
  cmd.AddValue ("appPeriod", "The period in seconds to be used by periodically transmitting applications", appPeriodSeconds);
  cmd.AddValue ("printEDs", "Whether or not to print a file containing the ED's positions", printEDs);
  cmd.AddValue ("gwPowerAveraging", "Gateway power averaging", gwPowerAveraging);
  cmd.AddValue ("HistoryAveraging", "History averaging", historyAveraging);
  cmd.AddValue ("HistoryRange", "History Range", historyRange);
  cmd.AddValue ("channelVariability", "Maximum channel random loss", channelVariability);
  cmd.Parse (argc, argv);

  Config::SetDefault( "ns3::AdrComponent::GwPowerAveraging", BooleanValue(gwPowerAveraging));
  Config::SetDefault( "ns3::AdrComponent::HistoryAveraging", BooleanValue(historyAveraging));
  Config::SetDefault( "ns3::AdrComponent::HistoryRange", UintegerValue(historyRange));

  // Set up logging
  LogComponentEnable ("ComplexLorawanNetworkExample", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
  // LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraMac", LOG_LEVEL_ALL);
  // LogComponentEnable("EndDeviceLoraMac", LOG_LEVEL_ALL);
  // LogComponentEnable("GatewayLoraMac", LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraMacHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraMacHeader", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
  //LogComponentEnable("EndDeviceStatus", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
  // LogComponentEnable("AdrComponent", LOG_LEVEL_ALL);

  //LogComponentEnableAll(LOG_PREFIX_NODE);
  //LogComponentEnableAll(LOG_PREFIX_TIME);

  /***********
   *  Setup  *
   ***********/

  // Compute the number of gateways
  nGateways = 3 * gatewayRings * gatewayRings - 3 * gatewayRings + 1;

  // Create the time value from the period
  Time appPeriod = Seconds (appPeriodSeconds);

  // Mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (0.0),
                                 "Y", DoubleValue (0.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  /************************
   *  Create the channel  *
   ************************/

  // Create the lora channel object
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7);

  Ptr<RandomPropagationLossModel> randomLoss = CreateObject<RandomPropagationLossModel> ();
  randomLoss->SetAttribute("Variable", StringValue(std::string("ns3::UniformRandomVariable[Min=0|Max=") + channelVariability + std::string("]")));
  loss->SetNext(randomLoss);

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  /************************
   *  Create the helpers  *
   ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LoraMacHelper
  LoraMacHelper macHelper = LoraMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking ("outcome_stats");

  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();

  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  /************************
   *  Create End Devices  *
   ************************/

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Assign a mobility model to each node
  mobility.Install (endDevices);

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j)
    {
      Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility->GetPosition ();
      position.z = 1.2;
      mobility->SetPosition (position);
    }

  // Create the LoraNetDevices of the end devices
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen =
    CreateObject<LoraDeviceAddressGenerator> (nwkId,nwkAddr);

  // Create the LoraNetDevices of the end devices
  macHelper.SetAddressGenerator (addrGen);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LoraMacHelper::ED);
  helper.Install (phyHelper, macHelper, endDevices);

  // Now end devices are connected to the channel

  /*********************
  *  Create Gateways  *
  *********************/

  // Create the gateway nodes (allocate them uniformely on the disc)
  NodeContainer gateways;
  gateways.Create (nGateways);

  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  // Make it so that nodes are at a certain height > 0
  allocator->Add (Vector (0.0, 0.0, 15.0));
  mobility.SetPositionAllocator (allocator);
  mobility.Install (gateways);


  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LoraMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  /**********************************************
  *  Set up the end device's spreading factor  *
  **********************************************/

  //sfQuantity = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
  *  Install applications on the end devices  *
  *********************************************/

  Time appStopTime = Seconds (simulationTime);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));
  appHelper.SetPacketSize (23);
  Ptr <RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> ("Min", DoubleValue (0), "Max", DoubleValue (10));
  ApplicationContainer appContainer = appHelper.Install (endDevices);

  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);

  /**********************
   * Print output files *
   *********************/
  if (printEDs)
    {
      PrintEndDevices (endDevices, gateways,
                       "src/lorawan/examples/endDevices_before.dat", helper);
    }

  /**************************
  *  Create Network Server  *
  ***************************/

  // Create the NS node
  NodeContainer networkServer;
  networkServer.Create (1);

  // Create a NS for the network
  nsHelper.SetEndDevices (endDevices);
  nsHelper.SetGateways (gateways);
  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install(gateways);

  /****************
   *  Simulation  *
  ****************/

  Time simulationStopTime = appStopTime + Minutes (15);
  Simulator::Stop (simulationStopTime);

  PrintPerformances (helper);

  Simulator::Run ();

  /**********************
   * Print output files *
   *********************/
  if (printEDs)
    {
      PrintEndDevices (endDevices, gateways,
                       "src/lorawan/examples/endDevices_after.dat", helper);
    }

  Simulator::Destroy ();

  // Print summary of whole simulation
  // helper.CountPhyPackets(Seconds (0), Time::Max ());

  return 0;
}
