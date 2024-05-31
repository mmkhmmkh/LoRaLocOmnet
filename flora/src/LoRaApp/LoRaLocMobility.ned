//
// Copyright (C) 2006 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


package flora.LoRaApp;

import inet.mobility.base.MobilityBase;

//
// This mobility module does nothing; it can be used for stationary nodes.
//
simple LoRaLocMobility extends MobilityBase
{
    parameters:
        double initialX @unit(m) = default(uniform(this.constraintAreaMinX, this.constraintAreaMaxX));
        double initialY @unit(m) = default(uniform(this.constraintAreaMinY, this.constraintAreaMaxY));
        double initialZ @unit(m) = default(0m);
        double initialLatitude @unit(deg) = default(nan deg);
        double initialLongitude @unit(deg) = default(nan deg);
        double initialAltitude @unit(m) = default(0m);
        double initialHeading @unit(deg) = default(0deg);
        double initialElevation @unit(deg) = default(0deg);
        double initialBank @unit(deg) = default(0deg);
        double step = default(0);
        bool initFromDisplayString = default(false); // enables one time initialization from the subject module's display string
        bool updateFromDisplayString = default(false); // enables continuous update from the subject module's display string for dragging and rotating it
        @class(LoRaLocMobility);
}

