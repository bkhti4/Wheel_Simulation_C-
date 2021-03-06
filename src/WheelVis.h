#ifndef WHEELVIS_H
#define WHEELVIS_H

#include <QMainWindow>
#include <QtCore>
#include <QLabel>
#include <QPainter>
#include <QMatrix2x2>
#include <QKeyEvent>
#include <string>
#include <thread>

#include <sstream>
#include <iomanip>
#include <iostream>

#include "CarSim.h"
#include "WheelSim.h"


QT_BEGIN_NAMESPACE
namespace Ui { class WheelVisualizer; }
QT_END_NAMESPACE

class WheelVis : public QMainWindow
{
    Q_OBJECT

public:
    WheelVis(QWidget *parent = nullptr);
    ~WheelVis();

    WheelSim wheel_left_front;
    WheelSim wheel_right_front;
    WheelSim wheel_left_rear;
    WheelSim wheel_right_rear;

    CarSim carSim;

private:
    int timerId;
    Ui::WheelVisualizer *ui;

    void keyPressEvent(QKeyEvent *event) override;

protected:
    void timerEvent(QTimerEvent *event);
};

#endif // WHEELVIS_H
