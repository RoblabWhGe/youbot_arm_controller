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
#include "JointController.h"
#include "ui_JointController.h"

JointController::JointController(Manipulator *manipulator, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::JointController)
{
    ui->setupUi(this);

    /* Store the given manipulator reference */
    this->manipulator = manipulator;

    /*Set minimum and maximum values to slider (based on robo freedom tests) */
    this->ui->axis1Slider->setMinimum(-168);
    this->ui->axis1Slider->setMaximum(165);
    this->ui->axis2Slider->setMinimum(-64);
    this->ui->axis2Slider->setMaximum(85);
    this->ui->axis3Slider->setMinimum(-141);
    this->ui->axis3Slider->setMaximum(145);
    this->ui->axis4Slider->setMinimum(-101);
    this->ui->axis4Slider->setMaximum(101);
    this->ui->axis5Slider->setMinimum(-161);
    this->ui->axis5Slider->setMaximum(155);
    this->ui->gripperSlider->setMinimum(0);
    this->ui->gripperSlider->setMaximum(25);

    /* Initialise time for GUI refreshing */
    this->guiRefreshTimer = new QTimer(this);
    connect(this->guiRefreshTimer, SIGNAL(timeout()), this, SLOT(on_gui_refresh_timeout()));

    /* Refresh GUI to obtain current slider position */
    this->refreshGuiState();
}

JointController::~JointController()
{
    delete ui;
}

void JointController::on_candleButton_clicked()
{
    this->manipulator->setPose(CANDLE_POSITION);
    this->refreshGuiState();
}

void JointController::on_homeButton_clicked()
{
    this->manipulator->setPose(HOME_POSITION);
    this->refreshGuiState();
}

void JointController::on_axis1Slider_valueChanged(int value)
{
    if (this->ui->axis1Slider->isEnabled())
    {
        this->manipulator->setAxis(1, value);
        this->ui->labelAxis1->setText(QString("%1").arg(value));
    }
}

void JointController::on_axis2Slider_valueChanged(int value)
{
    if (this->ui->axis2Slider->isEnabled())
    {
        this->manipulator->setAxis(2, value);
        this->ui->labelAxis2->setText(QString("%1").arg(value));
    }
}

void JointController::on_axis3Slider_valueChanged(int value)
{
    if (this->ui->axis3Slider->isEnabled())
    {
        this->manipulator->setAxis(3, value);
        this->ui->labelAxis3->setText(QString("%1").arg(value));
    }
}

void JointController::on_axis4Slider_valueChanged(int value)
{
    if (this->ui->axis4Slider->isEnabled())
    {
        this->manipulator->setAxis(4, value);
        this->ui->labelAxis4->setText(QString("%1").arg(value));
    }
}

void JointController::on_axis5Slider_valueChanged(int value)
{
    if (this->ui->axis5Slider->isEnabled())
    {
        this->manipulator->setAxis(5, value);
        this->ui->labelAxis5->setText(QString("%1").arg(value));
    }
}

void JointController::on_gripperSlider_valueChanged(int value)
{
    this->manipulator->setGripper(value);
}

void JointController::on_CloseGripper_clicked()
{
    this->manipulator->closeGripper();
    this->ui->gripperSlider->setValue(this->ui->gripperSlider->minimum());
}

void JointController::on_openGripper_clicked()
{
    this->manipulator->openGripper();
    this->ui->gripperSlider->setValue(this->ui->gripperSlider->maximum());
}

void JointController::directControlEnabled(bool enabled)
{
    this->ui->axis1Slider->setEnabled(enabled);
    this->ui->axis2Slider->setEnabled(enabled);
    this->ui->axis3Slider->setEnabled(enabled);
    this->ui->axis4Slider->setEnabled(enabled);
    this->ui->axis5Slider->setEnabled(enabled);
}

void JointController::on_gui_refresh_timeout()
{
    static QSlider *sliders[] = {this->ui->axis1Slider, this->ui->axis2Slider, this->ui->axis3Slider, this->ui->axis4Slider, this->ui->axis5Slider};
    static QLabel *labels[] = {this->ui->labelAxis1, this->ui->labelAxis2, this->ui->labelAxis3, this->ui->labelAxis4, this->ui->labelAxis5};

    vector<int> currentAnglesDeg;
    this->manipulator->getSensedAxis(currentAnglesDeg);

    for (int i = 0; i < 5; i++)
    {
        labels[i]->setText(QString("%1").arg(currentAnglesDeg[i]));
        sliders[i]->setValue(currentAnglesDeg[i]);
    }

    /* If we reached the target position no more refresh is nessecary */
    if (this->manipulator->positionReached())
    {
        this->guiRefreshTimer->stop();

        /* Activate slider */
        this->directControlEnabled(true);
    }
}

void JointController::refreshGuiState()
{
    /* Disable direct control of axis */
    this->directControlEnabled(false);

    /* Start timer for GUI refreshing */
    this->guiRefreshTimer->start(200);
}
