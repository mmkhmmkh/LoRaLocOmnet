package flora.simulations.mmkh;

import inet.applications.udpapp.UdpBasicApp;
import inet.node.inet.Router;
import inet.node.internetcloud.InternetCloud;
import flora.LoRaPhy.LoRaMedium;
import flora.LoraNode.LoRaNode;
import flora.LoraNode.LoRaGW;
import inet.node.inet.StandardHost;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth1G;

@license(LGPL);
network LoRaLocSimple
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfGateways = default(4);
        int networkSizeX = default(500);
        int networkSizeY = default(500);
        @display("bgb=562,417");
    submodules:
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=200,200");
        }
        loRaGW[numberOfGateways]: LoRaGW;
        LoRaMedium: LoRaMedium {
            @display("p=424,75");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=49,44");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=421,26");
        }
        internetCloud: InternetCloud {
            @display("p=231,44");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=312,44");
        }
        nsRouter: Router {
            @display("p=137,44");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}

