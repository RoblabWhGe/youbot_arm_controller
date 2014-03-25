/****************************************************************
 *
 * Copyright (c) 2014
 * All rights reserved.
 *
 * University of Applied Sciences Gelsenkirchen
 * Computer Science Department
 *
 * Author : Stefan Wilkes
 * Created: 21.03.2014
 *
 ***************************************************************/
#ifndef KINEMATICSSOLVER_H
#define KINEMATICSSOLVER_H

#include <eigen3/Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

/**
 * An object of this class implements the kinematics solver
 * for the youBot manipulator.
 *
 * @author Stefan Wilkes
 */
class KinematicsSolver
{
public:

    /**
     * Constructor:
     * Creates a new kinematics solver.
     */
    KinematicsSolver();

    /**
     * Calculates the TCP Matrix for a given robot state.
     * Forward kinematics.
     *
     * @param angles The actual axis values of the robot in radian.
     * @param tcp Vector which gives information about the posititon and
     *            orientation of the TCP (X, Y, Z, Roll, Pitch, Yaw)
     * @return true if the tcp state estimated successfully
     */
    bool forwardTransformation(VectorXd &angles, VectorXd &tcp);

    /**
     * Calculates the robot state for a given TCP
     * using the geometric inverse tranformation.
     *
     * @param tcp The TCP Vector for the desired world position
     *            (X, Y, Z, Roll, Pitch, Yaw)
     * @param angles A vector for storing the calculated angles
     * @return true if a possible state was calculated
     */
    bool inverseTransformation(VectorXd &tcp, VectorXd &angles);

private:

    /**
     * Implementation of the DH Transformation.
     *
     * @param theta Theta to next frame
     * @param d D to next frame
     * @param alpha Alpha to next frame
     * @param r R to next frame
     * @return Matrix with position and orientation for the next frame
     */
    Matrix4f dhTransformation(float theta, float d, float alpha, float r);
};

#endif // KINEMATICSSOLVER_H
