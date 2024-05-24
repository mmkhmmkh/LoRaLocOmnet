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

#include "NetworkServerApp.h"
//#include "inet/networklayer/ipv4/IPv4Datagram.h"
//#include "inet/networklayer/contract/ipv4/IPv4ControlInfo.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/common/L4PortTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/applications/base/ApplicationPacket_m.h"

#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/ipv4/Ipv4Header_m.h"

namespace flora {

Define_Module(NetworkServerApp);


void NetworkServerApp::initialize(int stage)
{
    printf("Inittttt!\n");
    if (stage == 0) {
        ASSERT(recvdPackets.size()==0);
        LoRa_ServerPacketReceived = registerSignal("LoRa_ServerPacketReceived");
        localPort = par("localPort");
        destPort = par("destPort");
        adrMethod = par("adrMethod").stdstringValue();
    } else if (stage == INITSTAGE_APPLICATION_LAYER) {
        startUDP();
        getSimulation()->getSystemModule()->subscribe("LoRa_AppPacketSent", this);
        evaluateADRinServer = par("evaluateADRinServer");
        adrDeviceMargin = par("adrDeviceMargin");
        receivedRSSI.setName("Received RSSI");
        totalReceivedPackets = 0;
        for(int i=0;i<6;i++)
        {
            counterUniqueReceivedPacketsPerSF[i] = 0;
            counterOfSentPacketsFromNodesPerSF[i] = 0;
        }
    }
}


void NetworkServerApp::startUDP()
{
    socket.setOutputGate(gate("socketOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), localPort);
}


void NetworkServerApp::handleMessage(cMessage *msg)
{
    if (msg->arrivedOn("socketIn")) {
        auto pkt = check_and_cast<Packet *>(msg);
        const auto &frame  = pkt->peekAtFront<LoRaMacFrame>();
        if (frame == nullptr)
            throw cRuntimeError("Header error type");
        //LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);
        if (simTime() >= getSimulation()->getWarmupPeriod())
        {
            totalReceivedPackets++;
        }
        updateKnownGateways(pkt);
        updateKnownNodes(pkt);
        processLoraMACPacket(pkt);
    }
    else if(msg->isSelfMessage()) {
        processScheduledPacket(msg);
    }
}

void NetworkServerApp::processLoraMACPacket(Packet *pk)
{
    const auto & frame = pk->peekAtFront<LoRaMacFrame>();
//    if(isPacketProcessed(frame))
//    {
//        printf("packet was processed.\n");
//        delete pk;
//        return;
//    }
    addPktToProcessingTable(pk);
}

void NetworkServerApp::finish()
{
    recordScalar("LoRa_NS_DER", double(counterUniqueReceivedPackets)/counterOfSentPacketsFromNodes);
    for(uint i=0;i<knownNodes.size();i++)
    {
        delete knownNodes[i].historyAllSNIR;
        delete knownNodes[i].historyAllRSSI;
        delete knownNodes[i].receivedSeqNumber;
        delete knownNodes[i].calculatedSNRmargin;
        recordScalar("Send ADR for node", knownNodes[i].numberOfSentADRPackets);
    }
    for(uint i=0;i<knownGateways.size();i++)
        {
            delete knownGateways[i].historySNIR;
            delete knownGateways[i].historyRSSI;
            delete knownGateways[i].receivedSN;
            delete knownGateways[i].nodeX;
            delete knownGateways[i].nodeY;
            delete knownGateways[i].deltaT;
            delete knownGateways[i].GW0historySNIR;
            delete knownGateways[i].GW0historyRSSI;
            delete knownGateways[i].GW0receivedSN;
            delete knownGateways[i].GW0nodeX;
            delete knownGateways[i].GW0nodeY;
            delete knownGateways[i].GW0deltaT;
            delete knownGateways[i].GW1historySNIR;
            delete knownGateways[i].GW1historyRSSI;
            delete knownGateways[i].GW1receivedSN;
            delete knownGateways[i].GW1nodeX;
            delete knownGateways[i].GW1nodeY;
            delete knownGateways[i].GW1deltaT;
            delete knownGateways[i].GW2historySNIR;
            delete knownGateways[i].GW2historyRSSI;
            delete knownGateways[i].GW2receivedSN;
            delete knownGateways[i].GW2nodeX;
            delete knownGateways[i].GW2nodeY;
            delete knownGateways[i].GW2deltaT;
            delete knownGateways[i].GW3historySNIR;
            delete knownGateways[i].GW3historyRSSI;
            delete knownGateways[i].GW3receivedSN;
            delete knownGateways[i].GW3nodeX;
            delete knownGateways[i].GW3nodeY;
            delete knownGateways[i].GW3deltaT;
        }
    for (std::map<int,int>::iterator it=numReceivedPerNode.begin(); it != numReceivedPerNode.end(); ++it)
    {
        const std::string stringScalar = "numReceivedFromNode " + std::to_string(it->first);
        recordScalar(stringScalar.c_str(), it->second);
    }

//    receivedRSSI.recordAs("receivedRSSI");
    recordScalar("totalReceivedPackets", totalReceivedPackets);

    while(!receivedPackets.empty()) {
        receivedPackets.back().endOfWaiting->removeControlInfo();
        delete receivedPackets.back().rcvdPacket;
        if (receivedPackets.back().endOfWaiting && receivedPackets.back().endOfWaiting->isScheduled()) {
            cancelAndDelete(receivedPackets.back().endOfWaiting);
        }
        else
            delete receivedPackets.back().endOfWaiting;
        receivedPackets.pop_back();
    }

    knownGateways.clear();
    knownNodes.clear();
    receivedPackets.clear();

    recordScalar("counterUniqueReceivedPacketsPerSF SF7", counterUniqueReceivedPacketsPerSF[0]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF8", counterUniqueReceivedPacketsPerSF[1]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF9", counterUniqueReceivedPacketsPerSF[2]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF10", counterUniqueReceivedPacketsPerSF[3]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF11", counterUniqueReceivedPacketsPerSF[4]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF12", counterUniqueReceivedPacketsPerSF[5]);
    if (counterOfSentPacketsFromNodesPerSF[0] > 0)
        recordScalar("DER SF7", double(counterUniqueReceivedPacketsPerSF[0]) / counterOfSentPacketsFromNodesPerSF[0]);
    else
        recordScalar("DER SF7", 0);

    if (counterOfSentPacketsFromNodesPerSF[1] > 0)
        recordScalar("DER SF8", double(counterUniqueReceivedPacketsPerSF[1]) / counterOfSentPacketsFromNodesPerSF[1]);
    else
        recordScalar("DER SF8", 0);

    if (counterOfSentPacketsFromNodesPerSF[2] > 0)
        recordScalar("DER SF9", double(counterUniqueReceivedPacketsPerSF[2]) / counterOfSentPacketsFromNodesPerSF[2]);
    else
        recordScalar("DER SF9", 0);

    if (counterOfSentPacketsFromNodesPerSF[3] > 0)
        recordScalar("DER SF10", double(counterUniqueReceivedPacketsPerSF[3]) / counterOfSentPacketsFromNodesPerSF[3]);
    else
        recordScalar("DER SF10", 0);

    if (counterOfSentPacketsFromNodesPerSF[4] > 0)
        recordScalar("DER SF11", double(counterUniqueReceivedPacketsPerSF[4]) / counterOfSentPacketsFromNodesPerSF[4]);
    else
        recordScalar("DER SF11", 0);

    if (counterOfSentPacketsFromNodesPerSF[5] > 0)
        recordScalar("DER SF12", double(counterUniqueReceivedPacketsPerSF[5]) / counterOfSentPacketsFromNodesPerSF[5]);
    else
        recordScalar("DER SF12", 0);
}

bool NetworkServerApp::isPacketProcessed(const Ptr<const LoRaMacFrame> &pkt)
{

    for(const auto & elem : knownNodes) {
        if(elem.srcAddr == pkt->getTransmitterAddress()) {
            if(elem.lastSeqNoProcessed > pkt->getSequenceNumber()) {
                printf("Duplicate packet! src %d sn %d\n", elem.srcAddr.getInt(), pkt->getSequenceNumber());
                return true;
            }
        }
    }
    return false;
}


void NetworkServerApp::updateKnownGateways(Packet* pkt)
{
    const auto& networkHeader = getNetworkProtocolHeader(pkt);
    const L3Address& gwAddress = networkHeader->getSourceAddress();
    bool gwExist = false;
    for(auto &elem : knownGateways)
    {
        if(elem.ipAddr == gwAddress) {
            gwExist = true;
            break;
        }
    }

    if(gwExist == false)
    {
        knownGW newGW;
        newGW.ipAddr = gwAddress;
        int gwid = (gwAddress.toIpv4().getDByte(3)-9)/4;
        char tmp[100];
        newGW.historyRSSI = new cOutVector;
        sprintf(tmp, "Node RSSI GW %d", gwid);
        newGW.historyRSSI->setName(tmp);
        newGW.historySNIR = new cOutVector;
        sprintf(tmp, "Node SNIR GW %d", gwid);
        newGW.historySNIR->setName(tmp);
        newGW.receivedSN = new cOutVector;
        sprintf(tmp, "Node SNs GW %d", gwid);
        newGW.receivedSN->setName(tmp);
        newGW.nodeX = new cOutVector;
        sprintf(tmp, "Node CoordXs GW %d", gwid);
        newGW.nodeX->setName(tmp);
        newGW.nodeY = new cOutVector;
        sprintf(tmp, "Node CoordYs GW %d", gwid);
        newGW.nodeY->setName(tmp);
        newGW.deltaT = new cOutVector;
        sprintf(tmp, "Node DeltaT GW %d", gwid);
        newGW.deltaT->setName(tmp);

        newGW.GW0historyRSSI = new cOutVector;
        sprintf(tmp, "GW0 RSSI GW %d", gwid);
        newGW.GW0historyRSSI->setName(tmp);
        newGW.GW0historySNIR = new cOutVector;
        sprintf(tmp, "GW0 SNIR GW %d", gwid);
        newGW.GW0historySNIR->setName(tmp);
        newGW.GW0receivedSN = new cOutVector;
        sprintf(tmp, "GW0 SNs GW %d", gwid);
        newGW.GW0receivedSN->setName(tmp);
        newGW.GW0nodeX = new cOutVector;
        sprintf(tmp, "GW0 CoordXs GW %d", gwid);
        newGW.GW0nodeX->setName(tmp);
        newGW.GW0nodeY = new cOutVector;
        sprintf(tmp, "GW0 CoordYs GW %d", gwid);
        newGW.GW0nodeY->setName(tmp);
        newGW.GW0deltaT = new cOutVector;
        sprintf(tmp, "GW0 DeltaT GW %d", gwid);
        newGW.GW0deltaT->setName(tmp);

        newGW.GW1historyRSSI = new cOutVector;
        sprintf(tmp, "GW1 RSSI GW %d", gwid);
        newGW.GW1historyRSSI->setName(tmp);
        newGW.GW1historySNIR = new cOutVector;
        sprintf(tmp, "GW1 SNIR GW %d", gwid);
        newGW.GW1historySNIR->setName(tmp);
        newGW.GW1receivedSN = new cOutVector;
        sprintf(tmp, "GW1 SNs GW %d", gwid);
        newGW.GW1receivedSN->setName(tmp);
        newGW.GW1nodeX = new cOutVector;
        sprintf(tmp, "GW1 CoordXs GW %d", gwid);
        newGW.GW1nodeX->setName(tmp);
        newGW.GW1nodeY = new cOutVector;
        sprintf(tmp, "GW1 CoordYs GW %d", gwid);
        newGW.GW1nodeY->setName(tmp);
        newGW.GW1deltaT = new cOutVector;
        sprintf(tmp, "GW1 DeltaT GW %d", gwid);
        newGW.GW1deltaT->setName(tmp);

        newGW.GW2historyRSSI = new cOutVector;
        sprintf(tmp, "GW2 RSSI GW %d", gwid);
        newGW.GW2historyRSSI->setName(tmp);
        newGW.GW2historySNIR = new cOutVector;
        sprintf(tmp, "GW2 SNIR GW %d", gwid);
        newGW.GW2historySNIR->setName(tmp);
        newGW.GW2receivedSN = new cOutVector;
        sprintf(tmp, "GW2 SNs GW %d", gwid);
        newGW.GW2receivedSN->setName(tmp);
        newGW.GW2nodeX = new cOutVector;
        sprintf(tmp, "GW2 CoordXs GW %d", gwid);
        newGW.GW2nodeX->setName(tmp);
        newGW.GW2nodeY = new cOutVector;
        sprintf(tmp, "GW2 CoordYs GW %d", gwid);
        newGW.GW2nodeY->setName(tmp);
        newGW.GW2deltaT = new cOutVector;
        sprintf(tmp, "GW2 DeltaT GW %d", gwid);
        newGW.GW2deltaT->setName(tmp);

        newGW.GW3historyRSSI = new cOutVector;
        sprintf(tmp, "GW3 RSSI GW %d", gwid);
        newGW.GW3historyRSSI->setName(tmp);
        newGW.GW3historySNIR = new cOutVector;
        sprintf(tmp, "GW3 SNIR GW %d", gwid);
        newGW.GW3historySNIR->setName(tmp);
        newGW.GW3receivedSN = new cOutVector;
        sprintf(tmp, "GW3 SNs GW %d", gwid);
        newGW.GW3receivedSN->setName(tmp);
        newGW.GW3nodeX = new cOutVector;
        sprintf(tmp, "GW3 CoordXs GW %d", gwid);
        newGW.GW3nodeX->setName(tmp);
        newGW.GW3nodeY = new cOutVector;
        sprintf(tmp, "GW3 CoordYs GW %d", gwid);
        newGW.GW3nodeY->setName(tmp);
        newGW.GW3deltaT = new cOutVector;
        sprintf(tmp, "GW3 DeltaT GW %d", gwid);
        newGW.GW3deltaT->setName(tmp);


        knownGateways.push_back(newGW);
    }
}

void NetworkServerApp::updateKnownNodes(Packet* pkt)
{
    const auto & frame = pkt->peekAtFront<LoRaMacFrame>();
    bool nodeExist = false;
    for(auto &elem : knownNodes)
    {
        if(elem.srcAddr == frame->getTransmitterAddress()) {
            nodeExist = true;
            if(elem.lastSeqNoProcessed < frame->getSequenceNumber()) {
                elem.lastSeqNoProcessed = frame->getSequenceNumber();
            }
            break;
        }
    }

    if(nodeExist == false)
    {
        knownNode newNode;
        newNode.srcAddr= frame->getTransmitterAddress();
        printf("New Node! src %d\n", newNode.srcAddr.getInt());
        newNode.lastSeqNoProcessed = frame->getSequenceNumber();
        newNode.framesFromLastADRCommand = 0;
        newNode.numberOfSentADRPackets = 0;
        newNode.historyAllSNIR = new cOutVector;
        newNode.historyAllSNIR->setName("Vector of SNIR per node");
        //newNode.historyAllSNIR->record(pkt->getSNIR());
        newNode.historyAllSNIR->record(math::fraction2dB(frame->getSNIR()));
        newNode.historyAllRSSI = new cOutVector;
        newNode.historyAllRSSI->setName("Vector of RSSI per node");
        newNode.historyAllRSSI->record(frame->getRSSI());
        newNode.receivedSeqNumber = new cOutVector;
        newNode.receivedSeqNumber->setName("Received Sequence number");
        newNode.calculatedSNRmargin = new cOutVector;
        newNode.calculatedSNRmargin->setName("Calculated SNRmargin in ADR");
        knownNodes.push_back(newNode);
    }
}

void NetworkServerApp::addPktToProcessingTable(Packet* pkt)
{
    const auto & frame = pkt->peekAtFront<LoRaMacFrame>();
    bool packetExists = false;
    for(auto &elem : receivedPackets)
    {
        const auto &frameAux = elem.rcvdPacket->peekAtFront<LoRaMacFrame>();
        if(frameAux->getTransmitterAddress() == frame->getTransmitterAddress() && frameAux->getSequenceNumber() == frame->getSequenceNumber())
        {
            packetExists = true;
            const auto& networkHeader = getNetworkProtocolHeader(pkt);
            const L3Address& gwAddress = networkHeader->getSourceAddress();
//            printf("from %s\n", gwAddress.str().c_str());
            elem.possibleGateways.emplace_back(gwAddress, frame->getSNIR(), frame->getRSSI(), simTime().raw());
            delete pkt;
            break;
        }
    }
    if(packetExists == false)
    {
        receivedPacket rcvPkt;
        rcvPkt.rcvdPacket = pkt;
        rcvPkt.endOfWaiting = new cMessage("endOfWaitingWindow");
        rcvPkt.endOfWaiting->setControlInfo(pkt);
        const auto& networkHeader = getNetworkProtocolHeader(pkt);
        const L3Address& gwAddress = networkHeader->getSourceAddress();
//        printf("sn %d from %d\n", frame->getSequenceNumber(), (gwAddress.toIpv4().getDByte(3)-9)/4);
        rcvPkt.possibleGateways.emplace_back(gwAddress, frame->getSNIR(), frame->getRSSI(), simTime().raw());
        EV << "Added " << gwAddress << " " << frame->getSNIR() << " " << frame->getRSSI() << endl;
        scheduleAt(simTime() + 1.2, rcvPkt.endOfWaiting);
        receivedPackets.push_back(rcvPkt);
    }
}

void NetworkServerApp::processScheduledPacket(cMessage* selfMsg)
{
    auto pkt = check_and_cast<Packet *>(selfMsg->removeControlInfo());
    const auto & frame = pkt->peekAtFront<LoRaMacFrame>();

    if (simTime() >= getSimulation()->getWarmupPeriod())
    {
        counterUniqueReceivedPacketsPerSF[frame->getLoRaSF()-7]++;
    }
    L3Address pickedGateway;
    double SNIRinGW = -99999999999;
    double RSSIinGW = -99999999999;
    int packetNumber;
    int nodeNumber;
    for(uint i=0;i<receivedPackets.size();i++)
    {
        const auto &frameAux = receivedPackets[i].rcvdPacket->peekAtFront<LoRaMacFrame>();
        if(frameAux->getTransmitterAddress() == frame->getTransmitterAddress() && frameAux->getSequenceNumber() == frame->getSequenceNumber())        {
            packetNumber = i;
            nodeNumber = frame->getTransmitterAddress().getInt();
            if (numReceivedPerNode.count(nodeNumber-1)>0)
            {
                ++numReceivedPerNode[nodeNumber-1];
            } else {
                numReceivedPerNode[nodeNumber-1] = 1;
            }

            const auto rcvAppPacket = getNodeData(receivedPackets[i].rcvdPacket);
            Coord* coords = new Coord(rcvAppPacket->getX(), rcvAppPacket->getY(), 0);
            int64_t arrival = rcvAppPacket->getCreationTime();

            int64_t minDeltaT = INT64_MAX;
            for(uint j=0;j<receivedPackets[i].possibleGateways.size();j++) {
                int64_t deltaT = std::get<3>(receivedPackets[i].possibleGateways[j]) - arrival;
                if (deltaT < minDeltaT) {
                    minDeltaT = deltaT;
                }
            }

            for(uint j=0;j<receivedPackets[i].possibleGateways.size();j++)
            {
                const L3Address& gwAddress = std::get<0>(receivedPackets[i].possibleGateways[j]);

                int nodeId = frameAux->getTransmitterAddress().getInt();
                if (!finished) {
                    if (nodeId == 1) {
                        printf("node ");
                    } else {
                        printf("GW %d ", nodeId-2);
                    }
                    printf("sn %d from %d rssi %lf coord (%lf, %lf)\n", frameAux->getSequenceNumber(), (std::get<0>(receivedPackets[i].possibleGateways[j]).toIpv4().getDByte(3)-9)/4, std::get<2>(receivedPackets[i].possibleGateways[j]), coords->x, coords->y);
                }

                for(uint k=0; k < knownGateways.size(); k++) {
//                    if (knownGateways[k].ipAddr == gwAddress && !(coords->x < 0.1 && coords->y > 999.9)) {
                    if (knownGateways[k].ipAddr == gwAddress && !finished) {
                        switch(nodeId) {
                            case 1:
                                if (!(coords->x < -99.9 && coords->y > 1099.9)) {
                                    knownGateways[k].receivedSN->record(frameAux->getSequenceNumber());
                                    knownGateways[k].nodeX->record(coords->x);
                                    knownGateways[k].nodeY->record(coords->y);
                                    knownGateways[k].historyRSSI->record(std::get<2>(receivedPackets[i].possibleGateways[j]));
                                    knownGateways[k].historySNIR->record(std::get<1>(receivedPackets[i].possibleGateways[j]));
                                    knownGateways[k].deltaT->record(std::get<3>(receivedPackets[i].possibleGateways[j]) - arrival - minDeltaT);
                                } else {
                                    finished = true;
                                }
                            break;
                            case 2:
                            knownGateways[k].GW0receivedSN->record(frameAux->getSequenceNumber());
                            knownGateways[k].GW0nodeX->record(coords->x);
                            knownGateways[k].GW0nodeY->record(coords->y);
                            knownGateways[k].GW0historyRSSI->record(std::get<2>(receivedPackets[i].possibleGateways[j]));
                            knownGateways[k].GW0historySNIR->record(std::get<1>(receivedPackets[i].possibleGateways[j]));
                            knownGateways[k].GW0deltaT->record(std::get<3>(receivedPackets[i].possibleGateways[j]) - arrival - minDeltaT);
                            break;
                            case 3:
                            knownGateways[k].GW1receivedSN->record(frameAux->getSequenceNumber());
                            knownGateways[k].GW1nodeX->record(coords->x);
                            knownGateways[k].GW1nodeY->record(coords->y);
                            knownGateways[k].GW1historyRSSI->record(std::get<2>(receivedPackets[i].possibleGateways[j]));
                            knownGateways[k].GW1historySNIR->record(std::get<1>(receivedPackets[i].possibleGateways[j]));
                            knownGateways[k].GW1deltaT->record(std::get<3>(receivedPackets[i].possibleGateways[j]) - arrival - minDeltaT);
                            break;
                            case 4:
                            knownGateways[k].GW2receivedSN->record(frameAux->getSequenceNumber());
                            knownGateways[k].GW2nodeX->record(coords->x);
                            knownGateways[k].GW2nodeY->record(coords->y);
                            knownGateways[k].GW2historyRSSI->record(std::get<2>(receivedPackets[i].possibleGateways[j]));
                            knownGateways[k].GW2historySNIR->record(std::get<1>(receivedPackets[i].possibleGateways[j]));
                            knownGateways[k].GW2deltaT->record(std::get<3>(receivedPackets[i].possibleGateways[j]) - arrival - minDeltaT);
                            break;
                            case 5:
                            knownGateways[k].GW3receivedSN->record(frameAux->getSequenceNumber());
                            knownGateways[k].GW3nodeX->record(coords->x);
                            knownGateways[k].GW3nodeY->record(coords->y);
                            knownGateways[k].GW3historyRSSI->record(std::get<2>(receivedPackets[i].possibleGateways[j]));
                            knownGateways[k].GW3historySNIR->record(std::get<1>(receivedPackets[i].possibleGateways[j]));
                            knownGateways[k].GW3deltaT->record(std::get<3>(receivedPackets[i].possibleGateways[j]) - arrival - minDeltaT);
                            break;


                        }
                        break;
                    }
                }

                if(SNIRinGW < std::get<1>(receivedPackets[i].possibleGateways[j]))
                {
                    RSSIinGW = std::get<2>(receivedPackets[i].possibleGateways[j]);
                    SNIRinGW = std::get<1>(receivedPackets[i].possibleGateways[j]);
                    pickedGateway = std::get<0>(receivedPackets[i].possibleGateways[j]);
                }
            }
        }
    }
    emit(LoRa_ServerPacketReceived, true);
    if (simTime() >= getSimulation()->getWarmupPeriod())
    {
        counterUniqueReceivedPackets++;
    }
//    receivedRSSI.collect(frame->getRSSI());
//    if(evaluateADRinServer)
//    {
//        evaluateADR(pkt, pickedGateway, SNIRinGW, RSSIinGW);
//    }
    delete receivedPackets[packetNumber].rcvdPacket;
    delete selfMsg;
    receivedPackets.erase(receivedPackets.begin()+packetNumber);
}


const Ptr<const LoRaAppPacket> NetworkServerApp::getNodeData(Packet* pkt)
{
    pkt->trimFront();
    pkt->removeAtFront<LoRaMacFrame>();
    return pkt->peekAtFront<LoRaAppPacket>();
}
//
//void NetworkServerApp::evaluateADR(Packet* pkt, L3Address pickedGateway, double SNIRinGW, double RSSIinGW)
//{
//    bool sendADR = false;
//    bool sendADRAckRep = false;
//    double SNRm; //needed for ADR
//    int nodeIndex;
//
//    pkt->trimFront();
//    auto frame = pkt->removeAtFront<LoRaMacFrame>();
//
//    const auto & rcvAppPacket = pkt->peekAtFront<LoRaAppPacket>();
//
//    if(rcvAppPacket->getOptions().getADRACKReq())
//    {
//        sendADRAckRep = true;
//    }
//
//    for(uint i=0;i<knownNodes.size();i++)
//    {
//        if(knownNodes[i].srcAddr == frame->getTransmitterAddress())
//        {
//            knownNodes[i].adrListSNIR.push_back(SNIRinGW);
//            knownNodes[i].historyAllSNIR->record(SNIRinGW);
//            knownNodes[i].historyAllRSSI->record(RSSIinGW);
//            knownNodes[i].receivedSeqNumber->record(frame->getSequenceNumber());
//            if(knownNodes[i].adrListSNIR.size() == 20) knownNodes[i].adrListSNIR.pop_front();
//            knownNodes[i].framesFromLastADRCommand++;
//
//            if(knownNodes[i].framesFromLastADRCommand == 20 || sendADRAckRep == true)
//            {
//                nodeIndex = i;
//                knownNodes[i].framesFromLastADRCommand = 0;
//                sendADR = true;
//                if(adrMethod == "max")
//                {
//                    SNRm = *max_element(knownNodes[i].adrListSNIR.begin(), knownNodes[i].adrListSNIR.end());
//                }
//                if(adrMethod == "avg")
//                {
//                    double totalSNR = 0;
//                    int numberOfFields = 0;
//                    for (std::list<double>::iterator it=knownNodes[i].adrListSNIR.begin(); it != knownNodes[i].adrListSNIR.end(); ++it)
//                    {
//                        totalSNR+=*it;
//                        numberOfFields++;
//                    }
//                    SNRm = totalSNR/numberOfFields;
//                }
//
//            }
//
//        }
//    }
//
//    if(sendADR || sendADRAckRep)
//    {
//        auto mgmtPacket = makeShared<LoRaAppPacket>();
//        mgmtPacket->setMsgType(TXCONFIG);
//
//        if(sendADR)
//        {
//            double SNRmargin;
//            double requiredSNR;
//            if(frame->getLoRaSF() == 7) requiredSNR = -7.5;
//            if(frame->getLoRaSF() == 8) requiredSNR = -10;
//            if(frame->getLoRaSF() == 9) requiredSNR = -12.5;
//            if(frame->getLoRaSF() == 10) requiredSNR = -15;
//            if(frame->getLoRaSF() == 11) requiredSNR = -17.5;
//            if(frame->getLoRaSF() == 12) requiredSNR = -20;
//
//            SNRmargin = SNRm - requiredSNR - adrDeviceMargin;
//            knownNodes[nodeIndex].calculatedSNRmargin->record(SNRmargin);
//            int Nstep = round(SNRmargin/3);
//            LoRaOptions newOptions;
//
//            // Increase the data rate with each step
//            int calculatedSF = frame->getLoRaSF();
//            while(Nstep > 0 && calculatedSF > 7)
//            {
//                calculatedSF--;
//                Nstep--;
//            }
//
//            // Decrease the Tx power by 3 for each step, until min reached
//            double calculatedPowerdBm = math::mW2dBmW(frame->getLoRaTP()) + 30;
//            while(Nstep > 0 && calculatedPowerdBm > 2)
//            {
//                calculatedPowerdBm-=3;
//                Nstep--;
//            }
//            if(calculatedPowerdBm < 2) calculatedPowerdBm = 2;
//
//            // Increase the Tx power by 3 for each step, until max reached
//            while(Nstep < 0 && calculatedPowerdBm < 14)
//            {
//                calculatedPowerdBm+=3;
//                Nstep++;
//            }
//            if(calculatedPowerdBm > 14) calculatedPowerdBm = 14;
//
//            newOptions.setLoRaSF(calculatedSF);
//            newOptions.setLoRaTP(calculatedPowerdBm);
//            EV << calculatedSF << endl;
//            EV << calculatedPowerdBm << endl;
//            mgmtPacket->setOptions(newOptions);
//        }
//
//        if(simTime() >= getSimulation()->getWarmupPeriod() && sendADR == true)
//        {
//            knownNodes[nodeIndex].numberOfSentADRPackets++;
//        }
//
//        auto frameToSend = makeShared<LoRaMacFrame>();
//        frameToSend->setChunkLength(B(par("headerLength").intValue()));
//
//      //  LoRaMacFrame *frameToSend = new LoRaMacFrame("ADRPacket");
//
//        //frameToSend->encapsulate(mgmtPacket);
//        frameToSend->setReceiverAddress(frame->getTransmitterAddress());
//        //FIXME: What value to set for LoRa TP
//        //frameToSend->setLoRaTP(pkt->getLoRaTP());
//        frameToSend->setLoRaTP(math::dBmW2mW(14));
//        frameToSend->setLoRaCF(frame->getLoRaCF());
//        frameToSend->setLoRaSF(frame->getLoRaSF());
//        frameToSend->setLoRaBW(frame->getLoRaBW());
//
//        auto pktAux = new Packet("ADRPacket");
//        mgmtPacket->setChunkLength(B(par("headerLength").intValue()));
//
//        pktAux->insertAtFront(mgmtPacket);
//        pktAux->insertAtFront(frameToSend);
//        socket.sendTo(pktAux, pickedGateway, destPort);
//
//    }
//    //delete pkt;
//}

void NetworkServerApp::receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details)
{
    if (simTime() >= getSimulation()->getWarmupPeriod())
    {
        counterOfSentPacketsFromNodes++;
        counterOfSentPacketsFromNodesPerSF[value-7]++;
    }
}

} //namespace inet
