//
// Copyright (C) 2006 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#ifndef __INET_LORALOCMOBILITY_H
#define __INET_LORALOCMOBILITY_H

#include "inet/mobility/base/StationaryMobilityBase.h"
#include "inet/common/INETMath.h"

using namespace inet;

namespace flora {

/**
 * This mobility module does not move at all; it can be used for standalone stationary nodes.
 *
 * @ingroup mobility
 */
class LoRaLocMobility : public StationaryMobilityBase
{
  protected:
    bool updateFromDisplayString;
    double step;

  protected:
    virtual void initialize(int stage) override;
    virtual void refreshDisplay() const override;
    virtual void updateMobilityStateFromDisplayString();

  public:
    virtual Coord& getPosition();
    virtual void updatePosition();

};

} // namespace inet

#endif

