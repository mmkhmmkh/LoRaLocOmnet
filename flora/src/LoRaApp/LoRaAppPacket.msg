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

//
// TODO generated message class
//

import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;

namespace flora;

enum AppPacketType
{
    JOIN_REQUEST = 1;
    JOIN_REPLY = 2;
    DATA = 3;
    TXCONFIG = 4;
}

class LoRaOptions {
    double LoRaTP = -1;
    double LoRaCF = -1;
    int LoRaSF = -1;
    double LoRaBW = -1;
    int LoRaCR = -1;
    bool UseHeader = true;
    bool ADRACKReq = false;
}

class LoRaAppPacket extends inet::FieldsChunk {
    int msgType @enum(AppPacketType);
    int sampleMeasurement;
    int64_t creationTime;
    double x;
    double y;
    LoRaOptions options;
}
