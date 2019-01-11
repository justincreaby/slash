#include "encoderFunctions.h"
#include <iostream>
#include <cmath>

extern "C" {
    #include <rc_usefulincludes.h>
	#include <roboticscape.h>
}


double calcDistanceTravelled(int &oldPulseCount, double metersPerPulse, int encoderChannel)
{
    int newPulseCount = rc_get_encoder_pos(encoderChannel);
    int differenceInPulseCount = std::abs (oldPulseCount - newPulseCount);
    double distanceTravelledSinceLastTimeStep = (double) differenceInPulseCount * metersPerPulse;
    oldPulseCount = newPulseCount;
    return distanceTravelledSinceLastTimeStep;
}