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
#ifndef JOINTCONTROLLER_H
#define JOINTCONTROLLER_H

#include "Manipulator.h"
#include <QMainWindow>
#include <QTimer>

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

private:

    /** Member object for the GUI of the controller */
    Ui::JointController *ui;

    /** Member object for the manipulator */
    Manipulator *manipulator;

    /** Timer for GUI refreshing */
    QTimer *guiRefreshTimer;

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
};

#endif // JOINTCONTROLLER_H
