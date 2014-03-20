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
#include "JointController.h"
#include <QApplication>

int main(int argc, char **argv)
{
    /* Create a manipulator object with given configfile */
    Manipulator manipulator("youbot-manipulator", "../config");

    /* Create a new GUI window and pass the manipulator for controlling */
    QApplication app(argc, argv);
    JointController gui(&manipulator);
    //JointController gui(NULL);
    gui.show();

    return app.exec();
}
