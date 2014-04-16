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
#include "JointController.h"
#include "ui_JointController.h"
#include "OfflineManipulator.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QProgressDialog>
#include <QApplication>
#include <QThread>

JointController::JointController(Manipulator *manipulator, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::JointController)
{
    /* Init GUI */
    ui->setupUi(this);

    /* Store the given manipulator reference */
    this->manipulator = manipulator;

    /* Check if we were in offline mode */
    if (typeid(*manipulator) == typeid(OfflineManipulator))
    {
        this->setWindowTitle(this->windowTitle() + " (Offline mode)");
    }

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

    /* Initialise timer for GUI refreshing */
    this->guiRefreshTimer = new QTimer(this);
    this->connect(this->guiRefreshTimer, SIGNAL(timeout()), this, SLOT(guiRefreshTimeout()));

    /* Initialise timer for automatic pose controlling */
    this->automaticModeTimer = new QTimer(this);
    this->connect(this->automaticModeTimer, SIGNAL(timeout()), this, SLOT(automaticModeTimeout()));
    this->automaticModePoseIndex = 0;

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
        this->readOutAbsolutePosition();
    }
}

void JointController::on_axis2Slider_valueChanged(int value)
{
    if (this->ui->axis2Slider->isEnabled())
    {
        this->manipulator->setAxis(2, value);
        this->ui->labelAxis2->setText(QString("%1").arg(value));
        this->readOutAbsolutePosition();
    }
}

void JointController::on_axis3Slider_valueChanged(int value)
{
    if (this->ui->axis3Slider->isEnabled())
    {
        this->manipulator->setAxis(3, value);
        this->ui->labelAxis3->setText(QString("%1").arg(value));
        this->readOutAbsolutePosition();
    }
}

void JointController::on_axis4Slider_valueChanged(int value)
{
    if (this->ui->axis4Slider->isEnabled())
    {
        this->manipulator->setAxis(4, value);
        this->ui->labelAxis4->setText(QString("%1").arg(value));
        this->readOutAbsolutePosition();
    }
}

void JointController::on_axis5Slider_valueChanged(int value)
{
    if (this->ui->axis5Slider->isEnabled())
    {
        this->manipulator->setAxis(5, value);
        this->ui->labelAxis5->setText(QString("%1").arg(value));
        this->readOutAbsolutePosition();
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

void JointController::guiRefreshTimeout()
{
    /* Refresh axis group */
    this->readOutAxisPositions();

    /* Refresh tcp position */
    this->readOutAbsolutePosition();

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

void JointController::readOutAbsolutePosition()
{
    static QLineEdit *text[] = {this->ui->editX, this->ui->editY, this->ui->editZ, this->ui->editRoll, this->ui->editPitch, this->ui->editYaw};

    VectorXd tcp;
    bool validData = this->manipulator->getSensedPosition(tcp);

    for (int i = 0; i < 6; i++)
    {
        if (validData)
        {
            int factor = (i < 3) ? 100 : 1;
            int precission = (i < 3) ? 2 : 4;
            text[i]->setText(QString("%1").arg(tcp[i] * factor, 0, 'f', precission));
        }
        else
        {
            text[i]->setText(QString(""));
        }
    }
}

void JointController::readOutAxisPositions()
{
    static QSlider *sliders[] = {this->ui->axis1Slider, this->ui->axis2Slider, this->ui->axis3Slider, this->ui->axis4Slider, this->ui->axis5Slider};
    static QLabel *labels[] = {this->ui->labelAxis1, this->ui->labelAxis2, this->ui->labelAxis3, this->ui->labelAxis4, this->ui->labelAxis5};

    VectorXi currentAnglesDeg(ARMJOINTS);
    this->manipulator->getSensedAxis(currentAnglesDeg);

    for (int i = 0; i < ARMJOINTS; i++)
    {
        labels[i]->setText(QString("%1").arg(currentAnglesDeg[i]));
        sliders[i]->setValue(currentAnglesDeg[i]);
    }
}

void JointController::on_sendButton_clicked()
{
    VectorXd tcp(6);
    tcp << this->ui->editX->text().toDouble() / 100.,
           this->ui->editY->text().toDouble() / 100.,
           this->ui->editZ->text().toDouble() / 100.,
           this->ui->editRoll->text().toDouble(),
           this->ui->editPitch->text().toDouble(),
           this->ui->editYaw->text().toDouble();
    bool success = this->manipulator->setPose(tcp);

    if (success)
    {
        this->refreshGuiState();
    }
    else
    {
        QMessageBox::warning(this, "Kinematics solver", "Can't set position to youBot.", QMessageBox::Ok);
    }
}

void JointController::on_loadButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open youBot pose file"), "", tr("*.ybposes"));

    if (!fileName.isEmpty())
    {
        QFile poseFile(fileName);

        if (poseFile.open(QFile::ReadOnly))
        {
           QTextStream poseStream(&poseFile);
           QString line = poseStream.readLine();

           this->storedAnglePositions.clear();

           if (line == QString("#POSITIONS"))
           {
              this->parsePositionStream(poseStream);
           }
           else if (line == QString("#ANGLES"))
           {
              this->parseAngleStram(poseStream);
           }
           else
           {
               QMessageBox::warning(this, "Open youBot pose file",
                        "Can't parse input file\nMissing operation mode in first line.", QMessageBox::Ok);
           }
           poseFile.close();
        }
        else
        {
            QMessageBox::warning(this, "Open youBot pose file", "Can't open input file", QMessageBox::Ok);
        }
    }
}

void JointController::parsePositionStream(QTextStream &positionStream)
{
    QProgressDialog progress("Parsing positions...\nThis can take a while in case of transformations.", "Cancel", 0, 100, this);
    progress.setWindowModality(Qt::WindowModal);

    qint64 pos = positionStream.pos();
    int stream_size = positionStream.readAll().size();
    positionStream.seek(pos);

    while (!positionStream.atEnd())
    {
        QString line = positionStream.readLine();
        QStringList positions = line.split(" ");
        if (positions.size() == 6)
        {
            VectorXd position(6);
            VectorXd angles;
            position << positions[0].toDouble(),
                        positions[1].toDouble(),
                        positions[2].toDouble(),
                        positions[3].toDouble(),
                        positions[4].toDouble(),
                        positions[5].toDouble();

            manipulator->prePlanMotion(position, angles);
            this->savePoseToInternalMemory(angles);
        }
        else
        {
            QMessageBox::warning(this, "Open youBot pose file", "Parsing error", QMessageBox::Ok);
        }
        progress.setValue((int)(positionStream.pos() * 100 / stream_size));
    }
}

void JointController::parseAngleStram(QTextStream &angleStream)
{
    QProgressDialog progress("Parsing angles...", "Cancel", 0, 100, this);
    progress.setWindowModality(Qt::WindowModal);

    qint64 pos = angleStream.pos();
    int stream_size = angleStream.readAll().size();
    angleStream.seek(pos);

    while (!angleStream.atEnd())
    {
        QString line = angleStream.readLine();
        QStringList angles = line.split(" ");
        if (angles.size() == 5)
        {
            VectorXd angleSet(5);
            angleSet << angles[0].toDouble(),
                        angles[1].toDouble(),
                        angles[2].toDouble(),
                        angles[3].toDouble(),
                        angles[4].toDouble(),

            this->savePoseToInternalMemory(angleSet);
        }
        else
        {
            QMessageBox::warning(this, "Open youBot pose file", "Parsing error", QMessageBox::Ok);
        }
        progress.setValue((int)(angleStream.pos() * 100 / stream_size));
    }
}


void JointController::on_addPoseButton_clicked()
{
    VectorXd axisState;
    this->manipulator->getSensedAxis(axisState);
    this->savePoseToInternalMemory(axisState);
}

void JointController::on_saveButton_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save youBot pose file"), "", tr("*.ybposes"));

    if (!fileName.isEmpty())
    {
        QFile poseFile(fileName);

        if (poseFile.open(QFile::WriteOnly))
        {
            QTextStream poseStream(&poseFile);
            poseStream << "#ANGLES\n";

            for (int i = 0; i < this->storedAnglePositions.size(); i++)
            {
                VectorXd angle = this->storedAnglePositions[i];
                poseStream << QString("%1 %2 %3 %4 %5\n").arg(
                    angle[0]).arg(angle[1]).arg(angle[2]).arg(angle[3]).arg(angle[4]);
            }
            poseFile.close();
        }
        else
        {
            QMessageBox::warning(this, "Save youBot pose file", "Can't write to input file", QMessageBox::Ok);
        }
    }
}

void JointController::savePoseToInternalMemory(VectorXd &angles)
{
    this->storedAnglePositions.push_back(angles);
    this->ui->poseLabel->setText(QString("Pose: %1 / %2").arg(
                this->automaticModePoseIndex).arg(this->storedAnglePositions.size()));
    this->ui->startButton->setEnabled(true);
}

void JointController::on_startButton_clicked()
{
    if (!this->automaticModeEnabled)
    {
        this->automaticModeTimer->start(200);
        this->automaticModeEnabled = true;
        this->ui->startButton->setText("Stop");
    }
    else
    {
        this->automaticModeTimer->stop();
        this->automaticModeEnabled = false;
        int result = QMessageBox::question(this, "Automatic mode stopped",
                "Go back to first pose or continue from this pose next time?", QMessageBox::Yes, QMessageBox::No);

        if (result == QMessageBox::No)
        {
            this->ui->startButton->setText("Pause");
        }
        else
        {
            this->ui->startButton->setText("Start");
            this->automaticModePoseIndex = 0;
            this->ui->poseLabel->setText(QString("Pose: %1 / %2").arg(
                        this->automaticModePoseIndex).arg(this->storedAnglePositions.size()));
        }
    }
}

void JointController::automaticModeTimeout()
{
    if (this->manipulator->positionReached())
    {
        if (this->automaticModePoseIndex < this->storedAnglePositions.size())
        {
            this->manipulator->setAxis(this->storedAnglePositions[this->automaticModePoseIndex]);
            this->automaticModePoseIndex++;
        }
        else
        {
            this->automaticModeTimer->stop();
            this->ui->startButton->setText("Start");
            this->automaticModeEnabled = false;
            this->automaticModePoseIndex = 0;
        }
        this->ui->poseLabel->setText(QString("Pose: %1 / %2").arg(
                    this->automaticModePoseIndex).arg(this->storedAnglePositions.size()));
    }
}
