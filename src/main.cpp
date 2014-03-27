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
#include <cstdlib>
#include <cstdio>
#include "Manipulator.h"
#include "OfflineManipulator.h"
#include "JointController.h"
#include <QApplication>
#include <QMessageBox>

/**
 * Main function:
 * Creates a new manipulator object if a connection is available. Otherwise
 * an offline manipualator is created for testing purpose.
 * The created manipualator object is given to the GUI object for controlling.
 *
 * @param argc Number of given arguments (not used)
 * @param argv List of given arguments (not used)
 * @return true if program quits successfully
 */
int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    Manipulator *manipulator;

    /* Try to connect to youBot, otherwise use offline simulator */
    try
    {
        /* Create a manipulator object with given configfile */
        manipulator = new Manipulator("youbot-manipulator", "../config");
    }
    catch (exception e)
    {
        QMessageBox::warning(NULL, "No connection to youBot...",
                    "Couldn't connect to youBot. Using simulation mode instead.", QMessageBox::Ok);
        manipulator = new OfflineManipulator();
    }

    /* Create a new GUI window and pass the manipulator for controlling */
    JointController gui(manipulator);
    gui.show();

    return app.exec();
}
