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
#ifndef JOINTCONTROLLER_H
#define JOINTCONTROLLER_H

#include "Manipulator.h"
#include <QMainWindow>
#include <QTimer>
#include <QTextStream>

namespace Ui
{
    class JointController;
}

/**
 * This is the controller unit for JointController GUI.
 * It takes a manipulator object and handles the communication
 * between the GUI actions and the youBot Arm.
 *
 * @author Stefan Wilkes
 */
class JointController : public QMainWindow
{
    Q_OBJECT
    
public:

    /**
     * Constructor:
     * Creates a new GUI controller and initialises the GUI itself.
     *
     * @param manipulator A reference for the manipulator
     * @param parent The parent GUI object (null, this is the main window)
     */
    explicit JointController(Manipulator *manipulator, QWidget *parent = 0);

    /**
     * Destructor.
     */
    ~JointController();
    
private slots:

    /**
     * Slot for changed slider value.
     *
     * @param value The new value of the slider
     */
    void on_axis1Slider_valueChanged(int value);

    /**
     * Slot for changed slider value.
     *
     * @param value The new value of the slider
     */
    void on_axis2Slider_valueChanged(int value);

    /**
     * Slot for changed slider value.
     *
     * @param value The new value of the slider
     */
    void on_axis3Slider_valueChanged(int value);

    /**
     * Slot for changed slider value.
     *
     * @param value The new value of the slider
     */
    void on_axis4Slider_valueChanged(int value);

    /**
     * Slot for changed slider value.
     *
     * @param value The new value of the slider
     */
    void on_axis5Slider_valueChanged(int value);

    /**
     * Slot for changed slider value.
     *
     * @param value The new value of the slider
     */
    void on_gripperSlider_valueChanged(int value);

    /**
     * Slot for specific button event.
     */
    void on_CloseGripper_clicked();

    /**
     * Slot for specific button event.
     */
    void on_openGripper_clicked();

    /**
     * Slot for specific button event.
     */
    void on_sendButton_clicked();

    /**
     * Slot for specific button event.
     */
    void on_candleButton_clicked();

    /**
     * Slot for specific button event.
     */
    void on_homeButton_clicked();

    /**
     * Slot for refresh timer timeout. The GUI
     * is refreshing till the manipulator reaches the target position.
     */
    void guiRefreshTimeout();

    /**
     * Shows the file open dialog to load stored poses.
     */
    void on_loadButton_clicked();

    /**
     * Adds the actual robot pose to the internal memory.
     */
    void on_addPoseButton_clicked();

    /**
     * Saves internal stored angle sets to a file (Angle format).
     */
    void on_saveButton_clicked();

    /**
     * Start / Stops automatic control mode.
     */
    void on_startButton_clicked();

    /**
     * Timer callback function for automatic control mode.
     */
    void automaticModeTimeout();

private:

    /** Member object for the GUI of the controller */
    Ui::JointController *ui;

    /** Member object for the manipulator */
    Manipulator *manipulator;

    /** Timer for GUI refreshing */
    QTimer *guiRefreshTimer;

    vector<VectorXd> storedAnglePositions;

    /** Memmber for automatic controlling. Index for the next pose */
    int automaticModePoseIndex;

    /** Flag indicating wheather automatic mode is enabled */
    bool automaticModeEnabled;

    /** Timer for automatic control mode */
    QTimer *automaticModeTimer;

    /**
     * Refreshes the GUI.
     * A timer is started which refreshs the GUI till the robot reaches its
     * desired position.
     */
    void refreshGuiState();

    /**
     * Enable or diables the direct axis controlling via the GUI
     *
     * @param enabled Flag wheather axis controlling should be enabled
     */
    void directControlEnabled(bool enabled);

    /**
     * Reads out the current axis state and sets the values to the slider.
     */
    void readOutAbsolutePosition();

    /**
     * Reads out the calculated tcp position and sets the values to the position control fields.
     */
    void readOutAxisPositions();

    /**
     * Stores the given angle set to internal memory for
     * automatic controlling.
     *
     * @param angles The angle set which should be stored.
     */
    void savePoseToInternalMemory(VectorXd &angles);
};

#endif // JOINTCONTROLLER_H
