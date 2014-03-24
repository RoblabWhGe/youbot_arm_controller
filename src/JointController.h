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

namespace Ui {
class JointController;
}

class JointController : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit JointController(Manipulator *manipulator, QWidget *parent = 0);
    ~JointController();
    
private slots:
    void on_candleButton_clicked();

    void on_axis1Slider_valueChanged(int value);

    void on_axis2Slider_valueChanged(int value);

    void on_homeButton_clicked();

    void on_axis3Slider_valueChanged(int value);

    void on_axis4Slider_valueChanged(int value);

    void on_axis5Slider_valueChanged(int value);

    void on_gripperSlider_valueChanged(int value);

    void on_CloseGripper_clicked();

    void on_openGripper_clicked();

    void guiRefreshTimeout();

private:
    Ui::JointController *ui;
    Manipulator *manipulator;
    QTimer *guiRefreshTimer;
    void refreshGuiState();
    void directControlEnabled(bool enabled);
    void readOutAbsolutePosition();
};

#endif // JOINTCONTROLLER_H
