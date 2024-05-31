//
// Copyright (C) 2006 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#include "LoRaLocMobility.h"

namespace flora {

Define_Module(LoRaLocMobility);

void LoRaLocMobility::initialize(int stage)
{
    StationaryMobilityBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
        updateFromDisplayString = par("updateFromDisplayString");
        step = par("step");
}

void LoRaLocMobility::refreshDisplay() const
{
    if (updateFromDisplayString) {
        const_cast<LoRaLocMobility *>(this)->updateMobilityStateFromDisplayString();
        DirectiveResolver directiveResolver(const_cast<LoRaLocMobility *>(this));
        auto text = format.formatString(&directiveResolver);
        getDisplayString().setTagArg("t", 0, text);
    }
    else
        StationaryMobilityBase::refreshDisplay();
}


void LoRaLocMobility::updatePosition() {
    double XYMIN = 1900;
    double XYMAX = 3100;
    XYMAX -= XYMIN;
    double x = lastPosition.x - XYMIN;
    double y = lastPosition.y - XYMIN;
    x += step;
    if (x > XYMAX) {
        x = 0;
        y += step;
        if (y > XYMAX) {
            x = XYMAX;
            y = XYMAX;
        }
    }
    x += XYMIN;
    y += XYMIN;
    lastPosition = Coord(x, y);
    emit(mobilityStateChangedSignal, const_cast<LoRaLocMobility *>(this));
}

Coord& LoRaLocMobility::getPosition() {
    return lastPosition;
}

void LoRaLocMobility::updateMobilityStateFromDisplayString()
{
    char *end;
    double depth;
    cDisplayString& displayString = subjectModule->getDisplayString();
    canvasProjection->computeCanvasPoint(lastPosition, depth);
    double x = strtod(displayString.getTagArg("p", 0), &end);
    double y = strtod(displayString.getTagArg("p", 1), &end);
    auto newPosition = canvasProjection->computeCanvasPointInverse(cFigure::Point(x, y), depth);
    if (lastPosition != newPosition) {
        lastPosition = newPosition;
        emit(mobilityStateChangedSignal, const_cast<LoRaLocMobility *>(this));
    }
    Quaternion swing;
    Quaternion twist;
    Coord vector = canvasProjection->computeCanvasPointInverse(cFigure::Point(0, 0), 1);
    vector.normalize();
    lastOrientation.getSwingAndTwist(vector, swing, twist);
    double oldAngle;
    Coord axis;
    twist.getRotationAxisAndAngle(axis, oldAngle);
    double newAngle = math::deg2rad(strtod(displayString.getTagArg("a", 0), &end));
    if (oldAngle != newAngle) {
        lastOrientation *= Quaternion(vector, newAngle - oldAngle);
        emit(mobilityStateChangedSignal, const_cast<LoRaLocMobility *>(this));
    }
}

} // namespace inet

