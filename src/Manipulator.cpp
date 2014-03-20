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
#include <cmath>
#include "Manipulator.h"

Manipulator::Manipulator(const string &name, const string &path)
{
    this->kukaArm = new YouBotManipulator(name, path);

    /* Enable control and initialise the arm */
    this->kukaArm->doJointCommutation();
    this->kukaArm->calibrateManipulator();

    /* The arm has 5 angles and no desired position */
    this->latestDesiredPosition.assign(ARMJOINTS, 0.);

    /* Drive arm to home position, to get definied value */
    this->setPose(HOME_POSITION);
}

bool Manipulator::setPose(STORED_POSES pose)
{
    bool poseSet = false;

    if (pose == HOME_POSITION)
    {
        vector<int> axis;
        axis.push_back(-168);
        axis.push_back(-64);
        axis.push_back(145);
        axis.push_back(-101);
        axis.push_back(-161);
        poseSet = this->setAxis(axis);
    }
    else if (pose == CANDLE_POSITION)
    {
        vector<double> axisRad(ARMJOINTS, 0.);
        poseSet = this->setAxis(axisRad);
    }

    return poseSet;
}

bool Manipulator::setAxis(vector<double> &targetAnglesRad)
{
    bool validVector = targetAnglesRad.size() == ARMJOINTS;

    if (validVector)
    {
        for (int i = 1; i <= ARMJOINTS && validVector; i++)
        {
            validVector = this->setAxis(i, targetAnglesRad[i - 1]);
        }
    }
    return validVector;
}

bool Manipulator::setAxis(vector<int> &targetAnglesDeg)
{
    bool validVector = targetAnglesDeg.size() == ARMJOINTS;

    if (validVector)
    {
        for (int i = 1; i <= ARMJOINTS && validVector; i++)
        {
            validVector = this->setAxis(i, targetAnglesDeg[i - 1]);
        }
    }
    return validVector;
}

bool Manipulator::setAxis(int jointIndex, double targetAngleRad)
{
    /* Convert angle to kuka angle (0 is left) */
    JointAngleSetpoint angle;
    angle.angle = ((jointIndex == 3) ? targetAngleRad - TOP_LIMIT_SD[jointIndex - 1] : targetAngleRad - BOTTOM_LIMIT_SD[jointIndex - 1]) * radian;

    return this->sendAxisCommandToManipulator(jointIndex, angle);
}

bool Manipulator::setAxis(int jointIndex, int targetAngleDeg)
{
    /* Convert angle to radian */
    double targetAngleRad = (M_PI / 180.) * targetAngleDeg;

    return this->setAxis(jointIndex, targetAngleRad);
}

void Manipulator::getSensedAxis(vector<double> &axisAnglesRad)
{
    vector<JointSensedAngle> data;
    this->kukaArm->getJointData(data);
    axisAnglesRad.clear();

    for (int i = 0; i < data.size(); i++)
    {
        /* Convert kuka angle to angle (0 is centered) */
        double angleRad = quantity_cast<double>(data[i].angle);
        angleRad = (i == 2) ? angleRad + TOP_LIMIT_SD[i] : angleRad + BOTTOM_LIMIT_SD[i];

        axisAnglesRad.push_back(angleRad);
    }
}

void Manipulator::getSensedAxis(vector<int> &axisAnglesDeg)
{
    /* Get current angles in radians */
    vector<double> axisAnglesRad;
    this->getSensedAxis(axisAnglesRad);

    /* Convert to degrees */
    axisAnglesDeg.clear();
    for (int i = 0; i < axisAnglesRad.size(); i++)
    {
        axisAnglesDeg.push_back((180. / M_PI) * axisAnglesRad[i]);
    }
}

bool Manipulator::sendAxisCommandToManipulator(int JointIndex, JointAngleSetpoint &targetAngle)
{
    /* Check if target angle is valid */
    bool validAngle = (targetAngle.angle > (BOTTOM_LIMIT_YB[JointIndex - 1] * radian)) && (targetAngle.angle < (TOP_LIMIT_YB[JointIndex - 1] * radian));

    /* Check if joint index is valid */
    bool validIndex = (JointIndex >= 1) && (JointIndex <= ARMJOINTS);

    if (validAngle && validIndex)
    {
        try
        {
            this->kukaArm->getArmJoint(JointIndex).setData(targetAngle);
            this->latestDesiredPosition[JointIndex - 1] = quantity_cast<double>(targetAngle.angle);
        }
        catch (std::exception e)
        {
            validAngle = false;
        }
    }
    return validAngle && validIndex;
}

void Manipulator::openGripper()
{
    this->kukaArm->getArmGripper().open();
}

void Manipulator::closeGripper()
{
    this->kukaArm->getArmGripper().close();
}

bool Manipulator::setGripper(int distance)
{
    bool spacingSet = true;
    GripperBarSpacingSetPoint barSpacing;
    barSpacing.barSpacing = (distance / 1000.) * meter;

    try
    {
        this->kukaArm->getArmGripper().setData(barSpacing);
    }
    catch (exception e)
    {
        spacingSet = false;
    }
    return spacingSet;
}

bool Manipulator::positionReached()
{
    bool positionReached = true;

    /* Get current axis states and compare to latest desired values (Epsilon of 1°) */
    vector<JointSensedAngle> data;
    this->kukaArm->getJointData(data);

    for (int i = 0; (i < ARMJOINTS) && positionReached; i++)
    {
        positionReached = abs(quantity_cast<double>(data[i].angle) - this->latestDesiredPosition[i]) < 0.01745;
    }
    return positionReached;
}
