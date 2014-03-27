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

    /* Get X, Y, Z, Roll, Pitch from tcp matrix. Yaw is always 0 thus hardware restrictions */
    double roll = (fabs(tcpMatrix(1,0)) >= 1) ? 0 : tcpMatrix(tcpMatrix(1,0)); // acos (l dot y)
    double pitch = (fabs(tcpMatrix(0,2)) >= 1) ? 0 : tcpMatrix(tcpMatrix(0,2)); // acos (n dot x)

    /* Calculate sign of roll and pitch  */
    double gamma = (tcpMatrix(2,2) <= -1) ? M_PI : ((tcpMatrix(2,2) >= 1) ? 0 : acos(tcpMatrix(2,2))); // acos (n dot z)
    if (gamma <= M_PI_2)
    {
        roll *= -1;
    }

    gamma = (tcpMatrix(1,1) <= -1) ? M_PI : ((tcpMatrix(1,1) >= 1) ? 0 : acos(tcpMatrix(1,1))); // acos (m dot y)
    if (gamma <= M_PI_2)
    {
        pitch *= -1;
    }
    tcp << tcpMatrix(0, 3), tcpMatrix(1, 3), tcpMatrix(2, 3), roll, pitch, 0;
}

bool KinematicsSolver::inverseTransformation(VectorXd &tcp, VectorXd &angles)
{
    /* Not implemented yet */
    return false;
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

