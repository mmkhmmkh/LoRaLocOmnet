//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef __LORANETWORK_NETWORKSERVERAPP_H_
#define __LORANETWORK_NETWORKSERVERAPP_H_

#include <omnetpp.h>
#include "inet/physicallayer/wireless/common/contract/packetlevel/RadioControlInfo_m.h"
#include <vector>
#include <tuple>
#include <algorithm>
#include "inet/common/INETDefs.h"

#include "LoRaMacControlInfo_m.h"
#include "LoRaMacFrame_m.h"
#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "../LoRaApp/LoRaAppPacket_m.h"
#include "inet/common/geometry/common/Coord.h"
#include <list>

namespace flora {

class knownNode
{
public:
    MacAddress srcAddr;
    int framesFromLastADRCommand;
    int lastSeqNoProcessed;
    int numberOfSentADRPackets;
    std::list<double> adrListSNIR;
    cOutVector *historyAllSNIR;
    cOutVector *historyAllRSSI;
    cOutVector *receivedSeqNumber;
    cOutVector *calculatedSNRmargin;
};

class knownGW
{
public:
    L3Address ipAddr;
    cOutVector *historySNIR;
    cOutVector *historyRSSI;
    cOutVector *receivedSN;
    cOutVector *nodeX;
    cOutVector *nodeY;
    cOutVector *deltaT;

    cOutVector *GW0historySNIR;
    cOutVector *GW0historyRSSI;
    cOutVector *GW0receivedSN;
    cOutVector *GW0nodeX;
    cOutVector *GW0nodeY;
    cOutVector *GW0deltaT;

    cOutVector *GW1historySNIR;
    cOutVector *GW1historyRSSI;
    cOutVector *GW1receivedSN;
    cOutVector *GW1nodeX;
    cOutVector *GW1nodeY;
    cOutVector *GW1deltaT;

    cOutVector *GW2historySNIR;
    cOutVector *GW2historyRSSI;
    cOutVector *GW2receivedSN;
    cOutVector *GW2nodeX;
    cOutVector *GW2nodeY;
    cOutVector *GW2deltaT;

    cOutVector *GW3historySNIR;
    cOutVector *GW3historyRSSI;
    cOutVector *GW3receivedSN;
    cOutVector *GW3nodeX;
    cOutVector *GW3nodeY;
    cOutVector *GW3deltaT;
};

class receivedPacket
{
public:
    Packet* rcvdPacket = nullptr;
    cMessage* endOfWaiting = nullptr;
    std::vector<std::tuple<L3Address, double, double, int64_t>> possibleGateways; // <address, sinr, rssi>
};

class NetworkServerApp : public cSimpleModule, cListener
{
  protected:
    std::vector<knownNode> knownNodes;
    std::vector<knownGW> knownGateways;
    std::vector<receivedPacket> receivedPackets;
    int localPort = -1, destPort = -1;
    std::vector<std::tuple<MacAddress, int>> recvdPackets;
    // state
    UdpSocket socket;
    cMessage *selfMsg = nullptr;
    int totalReceivedPackets;
    std::string adrMethod;
    double adrDeviceMargin;
    std::map<int, int> numReceivedPerNode;

  protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    void processLoraMACPacket(Packet *pk);
    void startUDP();
    void setSocketOptions();
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    bool isPacketProcessed(const Ptr<const LoRaMacFrame> &);
    void updateKnownGateways(Packet* pkt);
    void updateKnownNodes(Packet* pkt);
    void addPktToProcessingTable(Packet* pkt);
    void processScheduledPacket(cMessage* selfMsg);
    const Ptr<const LoRaAppPacket> getNodeData(Packet* pkt);
    void evaluateADR(Packet *pkt, L3Address pickedGateway, double SNIRinGW, double RSSIinGW);
    void receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details) override;
    bool evaluateADRinServer;

    cHistogram receivedRSSI;
  public:
    simsignal_t LoRa_ServerPacketReceived;
    int counterOfSentPacketsFromNodes = 0;
    int counterOfSentPacketsFromNodesPerSF[6];
    int counterUniqueReceivedPackets = 0;
    int counterUniqueReceivedPacketsPerSF[6];
    bool finished = false;
};
} //namespace inet
#endif
