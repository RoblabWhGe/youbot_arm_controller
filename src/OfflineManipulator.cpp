/****************************************************************
 *
 * Copyright (c) 2014
 * All rights reserved.
 *
 * University of Applied Sciences Gelsenkirchen
 * Computer Science Department
 *
 * Author : Stefan Wilkes
 * Created: 20.03.2014
 *
 ***************************************************************/
#include "OfflineManipulator.h"
#include "ybparams.h"

OfflineManipulator::OfflineManipulator()
{
    /* Same as for the real manipulator without arm initialisation */
    this->latestDesiredPosition = VectorXd(ARMJOINTS);
}

void OfflineManipulator::getSensedAxis(VectorXd &axisAnglesRad)
{
    /* Simply return the desired position as absolute angles */
    axisAnglesRad = this->latestDesiredPosition;

    for (int i = 0; i < ARMJOINTS; i++)
    {
        /* Convert kuka angle to angle (0 is centered) */
        axisAnglesRad[i] += (i == 2) ? TOP_LIMIT_SD[i] : BOTTOM_LIMIT_SD[i];
    }
}

bool OfflineManipulator::sendAxisCommandToManipulator(int jointIndex, JointAngleSetpoint &targetAngle)
{
    /* Simply store the desired value */
    this->latestDesiredPosition[jointIndex - 1] = quantity_cast<double>(targetAngle.angle);

    return true;
}

bool OfflineManipulator::positionReached()
{
    /* A bad simulator always reaches the position! */
    return true;
}

void OfflineManipulator::openGripper()
{
    /* Simply do nothing */
}

void OfflineManipulator::closeGripper()
{
    /* Simply do nothing */
}

bool OfflineManipulator::setGripper(int distance)
{
    /* It was successful, of course :) */
    return true;
}

