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
#ifndef OFFLINEMANIPULATOR_H
#define OFFLINEMANIPULATOR_H

#include "Manipulator.h"

/**
 * An object of this class is based on the manipualator object and overides
 * all hardware communications thus you can use it as an offline manipulator
 * for algorthmic testing purposes like the kinematics implementation.
 *
 * It assumes that all given positions are reached exactly and instantly.
 *
 * @author Stefan Wilkes
 */
class OfflineManipulator : public Manipulator
{
public:

    /**
     * Creates a new offline manipulator.
     */
    OfflineManipulator();

    /**
     * Overides the original communication function.
     * A position in the simulator is always reached.
     *
     * @return true
     */
    bool positionReached();

    /**
     * Overides the original communication function.
     * The sensed value is always the desired value.
     *
     * @param axisAnglesRad Vector for receiving the sensed values
     */
    void getSensedAxis(VectorXd &axisAnglesRad);

    /**
     * Overides the original communication function.
     * This function does actually nothing
     */
    void openGripper();

    /**
     * Overides the original communication function.
     * This function does actually nothing
     */
    void closeGripper();

    /**
     * Overides the original communication function.
     * This function does actually nothing.
     *
     * @return true
     */
    bool setGripper(int distance);

private:

    /**
     * Overides the original communication function.
     * This function simply store the desired axis values.
     *
     * @return true
     */
    bool sendAxisCommandToManipulator(int jointIndex, JointAngleSetpoint &targetAngle);
};

#endif // OFFLINEMANIPULATOR_H
