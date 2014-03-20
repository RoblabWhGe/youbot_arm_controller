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
#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <youbot/YouBotManipulator.hpp>
#include <vector>
#include "ybparams.h"

using namespace youbot;
using namespace std;

/**
 * Enumerator definition for stored positions
 */
typedef enum
{
    HOME_POSITION,
    CANDLE_POSITION
} STORED_POSES;

/**
 * An object of this class wraps the youBot API to a simple interface
 * for controlling the manipulator only. Axis values can be receivend and
 * sent in degree and radian values.
 *
 * Also carthesian coordinates can be set directly by using an
 * external kinematics solver.
 *
 * @author Stefan Wilkes
 */
class Manipulator
{

public:

    /**
     * Constructor:
     * Creates a new manipulator.
     *
     * @param name The name of the config file to load and pass to youBot api (without ".cfg")
     * @param path The path to the config file
     */
    Manipulator(const string &name, const string &path);

    /**
     * Sets a stored pose to the robot.
     *
     * @param pose Enumeration value for stored poses
     * @return true if pose could be set
     */
    bool setPose(STORED_POSES pose);

    /**
     *
     */
    bool setPose(double x, double y, double z, double roll, double pitch, double yaw);

    /**
     *
     */
    bool setAxis(vector<double> &targetAnglesRad);

    /**
     *
     */
    bool setAxis(vector<int> &targetAnglesDeg);

    /**
     *
     */
    bool setAxis(int jointIndex, double targetAngleRad);

    /**
     *
     */
    bool setAxis(int jointIndex, int targetAngleDeg);

    /**
     *
     */
    void getSensedAxis(vector<int> &axisAnglesDeg);

    /**
     *
     */
    void getSensedAxis(vector<double> &axisAnglesRad);

    /**
     *
     */
    void openGripper();

    /**
     *
     */
    void closeGripper();

    /**
     *
     */
    bool setGripper(int distance);

    /**
     *
     */
    bool positionReached();

private:

    /** */
    YouBotManipulator *kukaArm;

    /** */
    vector<double> latestDesiredPosition;

    /**
     *
     */
    bool sendAxisCommandToManipulator(int jointIndex, JointAngleSetpoint &targetAngle);
};

#endif // MANIPULATOR_H
