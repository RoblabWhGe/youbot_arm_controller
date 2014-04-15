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
#include "KinematicsSolver.h"
#include "ybparams.h"

KinematicsSolver::KinematicsSolver()
{

}

bool KinematicsSolver::forwardTransformation(VectorXd &angles, VectorXd &tcp)
{
    Matrix4f k[5];
    Matrix4f tcpMatrix;
    tcp = VectorXd(6);

    /* Calculate TCP matrix via DH Transformation */
    for (int i = 0; i < 5; i++)
    {
        k[i] = this->dhTransformation(angles[i] + DH_THETA[i], DH_D[i], DH_ALPHA[i], DH_R[i]);
    }
    tcpMatrix = k[0] * k[1] * k[2] * k[3] * k[4];

    /* Get Roll, Pitch from tcp matrix. (http://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel#Berechnung_aus_Rotationsmatrix) */
    double pitch = atan2(-tcpMatrix(2, 0), sqrt(pow(tcpMatrix(0, 0), 2) + pow(tcpMatrix(1, 0), 2)));
    double yaw;
    double roll;

    /* Singulary cases */
    if (fabs(pitch - M_PI_2) < 0.001)
    {
        yaw = 0;
        roll = atan2(tcpMatrix(0, 1), tcpMatrix(1, 1));
    }
    else if (fabs(pitch + M_PI_2) < 0.001)
    {
        yaw = 0;
        roll = -atan2(tcpMatrix(0, 1), tcpMatrix(1, 1));
    }
    else
    {
        yaw = atan2(tcpMatrix(1, 0) / cos(pitch), tcpMatrix(0, 0) / cos(pitch));
        roll = atan2(tcpMatrix(2, 1) / cos(pitch), tcpMatrix(2, 2) / cos(pitch));
    }

    tcp << tcpMatrix(0, 3), tcpMatrix(1, 3), tcpMatrix(2, 3), roll, pitch, yaw;

    /* The forward transformation always succeed :) */
    return true;
}

bool KinematicsSolver::inverseTransformation(VectorXd &tcp, VectorXd &angles)
{
    /* Call internal solver (Numeric, geometric, ...) */
    return this->inverseTransformationNumeric(tcp, angles);
}

Matrix4f KinematicsSolver::dhTransformation(float theta, float d, float alpha, float r)
{
    Matrix4f rotZ(4, 4);
    Matrix4f transZ(4, 4);
    Matrix4f rotX(4, 4);
    Matrix4f transX(4, 4);

    float cosTheta = cos(theta);
    float sinTheta = sin(theta);
    float cosAlpha = cos(alpha);
    float sinAlpha = sin(alpha);

    rotZ << cosTheta, -sinTheta, 0, 0,
            sinTheta, cosTheta , 0, 0,
            0       , 0        , 1, 0,
            0       , 0        , 0, 1;

    transZ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, d,
              0, 0, 0, 1;

    transX << 1, 0, 0, r,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    rotX << 1, 0       , 0,         0,
            0, cosAlpha, -sinAlpha, 0,
            0, sinAlpha,  cosAlpha, 0,
            0, 0       , 0,         1;

    return rotZ * transZ * transX * rotX;
}

bool KinematicsSolver::inverseTransformationNumeric(VectorXd &tcp, VectorXd &angles)
{
    /* Start iteration with candle postition */
    angles.resize(5);
    angles << 0., 0., 0., 0., 0.;

    /* Get distance for start position */
    VectorXd tcpCurrent;
    forwardTransformation(angles, tcpCurrent);
    double minDistance = this->calculateDistanceBetweenTCPS(tcpCurrent, tcp);

    /* Try to optimize till offset reaches 0.1 degree. Start iteration with 1 degree */
    for (double offset = 0.0174; offset > 0.0017; offset /= 2.0)
    {
        bool optimisationFound = true;
        while (optimisationFound)
        {
            bool optimisedAngle[5];

            /* Find best angle for all axis */
            for (int a = 0; a < 5; a++)
            {
                double distance;

                /* Guess we'll find an optimisation */
                optimisedAngle[a] = true;

                /* Check left angle */
                angles[a] -= offset;
                this->forwardTransformation(angles, tcpCurrent);
                distance = this->calculateDistanceBetweenTCPS(tcpCurrent, tcp);

                /* Optimization? Otherwise check right angle  */
                if (distance < minDistance && angles[a] > BOTTOM_LIMIT_SD[a])
                {
                    minDistance = distance;
                }
                else
                {
                    angles[a] += 2 * offset;
                    this->forwardTransformation(angles, tcpCurrent);
                    distance = this->calculateDistanceBetweenTCPS(tcpCurrent, tcp);

                    /* Optimization? Otherwise revert modified angle */
                    if (distance < minDistance && angles[a] < TOP_LIMIT_SD[a])
                    {
                        minDistance = distance;
                    }
                    else
                    {
                        angles[a] -= offset;
                        optimisedAngle[a] = false;
                    }
                }
            }
            /* Check if we have found any optimisation */
            optimisationFound = optimisedAngle[0] || optimisedAngle[1] ||
                    optimisedAngle[2] || optimisedAngle[3] || optimisedAngle[4];
        }
    }

    /* The numeric transformation always approximate a TCP pose */
    return true;
}

double KinematicsSolver::calculateDistanceBetweenTCPS(VectorXd &tcp1, VectorXd &tcp2)
{
    double distance;

    /* Ignore RPY and calculate euclydian distance between X, Y and Z */
    distance = sqrt(pow((tcp1[0] - tcp2[0]), 2) +
                    pow((tcp1[1] - tcp2[1]), 2) +
                    pow((tcp1[2] - tcp2[2]), 2));

    return distance;
}

void KinematicsSolver::convertTCPVectorToMatrix(VectorXd &tcpVector, Matrix4f &tcpMatrix)
{
    Matrix3f baseXYZ;
    Matrix3f eulerZYX;

    /* Convert TCP vector to TCP Matrix vial ZYX euler rotation */
    baseXYZ << 1, 0, 0,
               0, 1, 0,
               0, 0, 1;

    eulerZYX = AngleAxisf(tcpVector[5], Vector3f::UnitZ()) *
               AngleAxisf(tcpVector[4], Vector3f::UnitY()) *
               AngleAxisf(tcpVector[3], Vector3f::UnitX());
    baseXYZ *= eulerZYX;

    tcpMatrix << baseXYZ(0, 0), baseXYZ(0, 1), baseXYZ(0, 2), tcpVector[0],
                 baseXYZ(1, 0), baseXYZ(1, 1), baseXYZ(1, 2), tcpVector[1],
                 baseXYZ(2, 0), baseXYZ(2, 1), baseXYZ(2, 2), tcpVector[2],
                 0,             0,             0,             1;
}

