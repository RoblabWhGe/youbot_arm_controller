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
