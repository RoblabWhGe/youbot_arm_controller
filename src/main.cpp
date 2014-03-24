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
