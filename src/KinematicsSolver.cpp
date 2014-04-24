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
    return false;
}

bool KinematicsSolver::inverseTransformation(VectorXd &tcp, VectorXd &angles)
{
    return false;
}