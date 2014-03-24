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
#include "ybparams.h"

Manipulator::Manipulator()
{

}

Manipulator::Manipulator(const string &name, const string &path)
{
    this->kukaArm = new YouBotManipulator(name, path);

    /* Enable control and initialise the arm */
    this->kukaArm->doJointCommutation();
    this->kukaArm->calibrateManipulator();

    /* The arm has 5 angles and no desired position */
    this->latestDesiredPosition = VectorXd(ARMJOINTS);

    /* Drive arm to home position, to get definied value */
    this->setPose(HOME_POSITION);

    /* Create a new kinematics solver */
    this->solver = new KinematicsSolver();

    JointVelocitySetpoint data;
    data.angularVelocity = 0.001 * radian_per_second;
    this->kukaArm->getArmJoint(1).setData(data);
}

bool Manipulator::setPose(STORED_POSES pose)
{
    bool poseSet = false;

    if (pose == HOME_POSITION)
    {
        VectorXi axis(ARMJOINTS);
        axis << -168, -64, 145, -101, -161;
        poseSet = this->setAxis(axis);
    }
    else if (pose == CANDLE_POSITION)
    {
        VectorXd axisRad(ARMJOINTS);
        axisRad << 0, 0, 0, 0, 0;
        poseSet = this->setAxis(axisRad);
    }
    return poseSet;
}

bool Manipulator::setAxis(VectorXd &targetAnglesRad)
{
    bool validVector = true;

    for (int i = 1; i <= ARMJOINTS && validVector; i++)
    {
        validVector = this->setAxis(i, targetAnglesRad[i - 1]);
    }

    return validVector;
}

bool Manipulator::setAxis(VectorXi &targetAnglesDeg)
{
    bool validVector = true;

   for (int i = 1; i <= ARMJOINTS && validVector; i++)
    {
        validVector = this->setAxis(i, targetAnglesDeg[i - 1]);
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

void Manipulator::getSensedAxis(VectorXd &axisAnglesRad)
{
    vector<JointSensedAngle> data;
    this->kukaArm->getJointData(data);

    for (int i = 0; i < ARMJOINTS; i++)
    {
        /* Convert kuka angle to angle (0 is centered) */
        axisAnglesRad[i] = quantity_cast<double>(data[i].angle);
        axisAnglesRad[i] += (i == 2) ? TOP_LIMIT_SD[i] : BOTTOM_LIMIT_SD[i];
    }
}

void Manipulator::getSensedAxis(VectorXi &axisAnglesDeg)
{
    /* Get current angles in radians */
    VectorXd axisAnglesRad(ARMJOINTS);
    this->getSensedAxis(axisAnglesRad);

    /* Convert to degrees */
    for (int i = 0; i < ARMJOINTS; i++)
    {
        axisAnglesDeg[i] = ((180. / M_PI) * axisAnglesRad[i]);
    }
}

void Manipulator::getSensedPosition(VectorXd &tcp)
{
    /* Get current axis state */
    VectorXd angles(ARMJOINTS);
    this->getSensedAxis(angles);

    /* Do forward transformation */
    this->solver->forwardTransformation(angles, tcp);
}

bool Manipulator::sendAxisCommandToManipulator(int jointIndex, JointAngleSetpoint &targetAngle)
{
    /* Check if target angle is valid */
    bool validAngle = (targetAngle.angle > (BOTTOM_LIMIT_YB[jointIndex - 1] * radian)) && (targetAngle.angle < (TOP_LIMIT_YB[jointIndex - 1] * radian));

    /* Check if joint index is valid */
    bool validIndex = (jointIndex >= 1) && (jointIndex <= ARMJOINTS);

    if (validAngle && validIndex)
    {
        try
        {
            this->kukaArm->getArmJoint(jointIndex).setData(targetAngle);
            this->latestDesiredPosition[jointIndex - 1] = quantity_cast<double>(targetAngle.angle);
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
