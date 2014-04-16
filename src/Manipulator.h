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
#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <youbot/YouBotManipulator.hpp>
#include <eigen3/Eigen/Dense>
#include "KinematicsSolver.h"

using namespace youbot;
using namespace Eigen;

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
     * Sets a given pose to the robot.
     * The coordinates defines the position of the tcp in world
     * system. An external kinematics solver is used to calculate
     * the angles for a given pose.
     *
     * @param tcp The TCP Vector for the desired world position
     *            (X, Y, Z, Roll, Pitch, Yaw)
     * @return true if pose set successfully
     */
    bool setPose(VectorXd &tcp);

    /**
     * Sets the given target angles (radian) to the robot.
     * The vector must have defined all 5 axis values.
     * To set a single axis use overloaded function.
     *
     * @param targetAnglesRad Vector of 5 axis angles
     * @return true if angles set successfully
     */
    bool setAxis(VectorXd &targetAnglesRad);

    /**
     * Sets the given target angles (degree) to the robot.
     * The vector must have defined all 5 axis values.
     * To set a single axis use overloaded function.
     *
     * @param targetAnglesDeg Vector of 5 axis angles
     * @return true if angles set successfully
     */
    bool setAxis(VectorXi &targetAnglesDeg);

    /**
     * Sets a single target angle (radian) to the robot.
     *
     * @param jointIndex Index for joint which should be set (1 - 5)
     * @param targetAngleRad Angle to set
     * @return true if angle set successfully
     */
    bool setAxis(int jointIndex, double targetAngleRad);

    /**
     * Sets a single target angle (degree) to the robot.
     *
     * @param jointIndex Index for joint which should be set (1 - 5)
     * @param targetAngleRad Angle to set
     * @return true if angle set successfully
     */
    bool setAxis(int jointIndex, int targetAngleDeg);

    /**
     * Reads out the actual axis positions of the robot in degree.
     *
     * @param axisAngleDeg Vector which receives the readed values
     */
    void getSensedAxis(VectorXi &axisAnglesDeg);

    /**
     * Reads out the actual axis positions of the robot in radian.
     *
     * @param axisAngleRad Vector which receives the readed values
     */
    virtual void getSensedAxis(VectorXd &axisAnglesRad);

    /**
     * Reads out the actual pose of the TCP calculated by an external
     * kinematics solver.
     *
     * @param tcp Vector for storing the calculated pose
     * @return true if the position was calculated successfully
     */
    bool getSensedPosition(VectorXd &tcp);

    /**
     * Pre plans the motion for a given TCP.
     *
     * An external kinematics solver is used to calculate
     * the angles for a given pose.
     *
     * No command is send to the robot. This is only for
     * pre calculation of large position commands.
     *
     * @param tcp The TCP Vector for the desired world position
     *            (X, Y, Z, Roll, Pitch, Yaw)
     * @param angles The calculated angles which would be sent to the robot.
     * @return true if pose set successfully
     */
    bool prePlanMotion(VectorXd &tcp, VectorXd &angles);

    /**
     * Opens the gripper of the robot.
     */
    virtual void openGripper();

    /**
     * Closes the gripper of the robot.
     */
    virtual void closeGripper();

    /**
     * Sets the spacing of the gripper manually.
     *
     * @param distance Open space of the gripper in mm
     * @return true if the gripper spacing set successfully
     */
    virtual bool setGripper(int distance);

    /**
     * Checks if the robot has reached the latest given position.
     * The target space has a threshold of one degree.
     *
     * @return true if the robot has reached the position
     */
    virtual bool positionReached();

protected:

    /**
     * Constructor:
     * Creates an empty manipulator object for derivated classes, which doesn't
     * connect to the arm e.g. an offline simulator.
     */
    Manipulator();

    /**
     * Finally sends a processed target angle to the robot.
     *
     * @param jointIndex
     * @param targetAngle
     * @return
     */
    virtual bool sendAxisCommandToManipulator(int jointIndex, JointAngleSetpoint &targetAngle);

    /** Vector for latest desired position */
    VectorXd latestDesiredPosition;

    /** Member object for the kinematics solver */
    KinematicsSolver *solver;

private:

    /** Member Object for the youBot API for arm communication */
    YouBotManipulator *kukaArm;
};

#endif // MANIPULATOR_H
