/*
 * This file is part of youbot_arm_controller
 *
 * Copyright (c)2014 by Robotics Lab 
 * in the Computer Science Department of the 
 * University of Applied Science Gelsenkirchen
 * 
 * Author: Stefan Wilkes <stefan.wilkes@studmail.w-hs.de>
 *  
 * The package is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
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

