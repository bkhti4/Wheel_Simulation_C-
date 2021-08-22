#include "WheelVis.h"
#include "./ui_wheelvisualizer.h"

WheelVis::WheelVis(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::WheelVisualizer)
{
    std::ostringstream throttlePowerString;
    std::ostringstream brakePowerString;
    std::ostringstream steeringAngleString;
    std::ostringstream temperatureString;

    std::ostringstream leftFrontSlipString;
    std::ostringstream leftRearSlipString;
    std::ostringstream rightFrontSlipString;
    std::ostringstream rightRearSlipString;

    std::ostringstream leftFrontTempString;
    std::ostringstream leftRearTempString;
    std::ostringstream rightFrontTempString;
    std::ostringstream rightRearTempString;

    std::ostringstream leftFrontLongString;
    std::ostringstream leftRearLongString;
    std::ostringstream rightFrontLongString;
    std::ostringstream rightRearLongString;

    std::ostringstream leftFrontLatString;
    std::ostringstream leftRearLatString;
    std::ostringstream rightFrontLatString;
    std::ostringstream rightRearLatString;

    timerId = startTimer(100);
    ui->setupUi(this);

    QLabel *accStatus = ui->AccelerationLabel;
    accStatus->clear();
    accStatus->setText("Acceleration:  " + QString::fromStdString(std::to_string(carSim.acceleration)));


    QLabel *fXStatus = ui->LongitudnalLabel;
    fXStatus->clear();
    fXStatus->setText("Longitudnal Force:  " + QString::fromStdString(std::to_string(0.0)) + " N");


    QLabel *fYStatus = ui->LateralLabel;
    fYStatus->clear();
    fYStatus->setText("Lateral Force:  " + QString::fromStdString(std::to_string(0.0)) + " N");


    QLabel *thrtStatus = ui->ThrottleLabel;
    thrtStatus->clear();
    throttlePowerString << std::fixed << std::setprecision(2) << (carSim.throtlePower / carSim.maxThrotlePower);
    thrtStatus->setText("Throttle Power:  " + QString::fromStdString(throttlePowerString.str()) + " %");

    QLabel *brkStatus = ui->BrakeLabel;
    brkStatus->clear();
    brakePowerString << std::fixed << std::setprecision(2) << (carSim.brakePower / carSim.maxBrakePower);
    brkStatus->setText("Brake Power:  " + QString::fromStdString(brakePowerString.str()) + " %");

    QLabel *strAngStatus = ui->SteeringLabel;
    strAngStatus->clear();
    steeringAngleString << std::fixed << std::setprecision(2) << carSim.steeringAngle;
    strAngStatus->setText("Steering Angle:  " + QString::fromStdString(brakePowerString.str()) + " deg");

    QLabel *leftFSlipStatus = ui->leftFrontSlipQlabel;
    leftFSlipStatus->clear();
    leftFrontSlipString << std::fixed << std::setprecision(2) << 0.0;
    leftFSlipStatus->setText("Slip Angle:  " + QString::fromStdString(leftFrontSlipString.str()));
    QLabel *leftFLongStatus = ui->leftFrontLongQlabel;
    leftFLongStatus->clear();
    leftFrontLongString << std::fixed << std::setprecision(2) << 0.0;
    leftFLongStatus->setText("Longitudnal Force:  " + QString::fromStdString(leftFrontLongString.str()));

    QLabel *leftRSlipStatus = ui->leftRearSlipQlabel;
    leftRSlipStatus->clear();
    leftRearSlipString << std::fixed << std::setprecision(2) << 0.0;
    leftRSlipStatus->setText("Slip Angle:  " + QString::fromStdString(leftRearSlipString.str()));
    QLabel *leftRLongStatus = ui->leftRearLongQlabel;
    leftRLongStatus->clear();
    leftRearLongString << std::fixed << std::setprecision(2) << 0.0;
    leftRLongStatus->setText("Longitudnal Force:  " + QString::fromStdString(leftRearLongString.str()));

    QLabel *rightFSlipStatus = ui->rightFrontSlipQlabel;
    rightFSlipStatus->clear();
    rightFrontSlipString << std::fixed << std::setprecision(2) << 0.0;
    rightFSlipStatus->setText("Slip Angle:  " + QString::fromStdString(rightFrontSlipString.str()));
    QLabel *rightFLongStatus = ui->rightFrontLongQlabel;
    rightFLongStatus->clear();
    rightFrontLongString << std::fixed << std::setprecision(2) << 0.0;
    rightFLongStatus->setText("Longitudnal Force:  " + QString::fromStdString(rightFrontLongString.str()));

    QLabel *rightRSlipStatus = ui->rightRearSlipQlabel;
    rightRSlipStatus->clear();
    rightRearSlipString << std::fixed << std::setprecision(2) << 0.0;
    rightRSlipStatus->setText("Slip Angle:  " + QString::fromStdString(rightRearSlipString.str()));
    QLabel *rightRLongStatus = ui->rightRearLongQlabel;
    rightRLongStatus->clear();
    rightRearLongString << std::fixed << std::setprecision(2) << 0.0;
    rightRLongStatus->setText("Longitudnal Force:  " + QString::fromStdString(rightRearLongString.str()));


    QLabel *leftFLatStatus = ui->leftFrontLatQlabel;
    leftFLatStatus->clear();
    leftFrontLatString << std::fixed << std::setprecision(2) << 0.0;
    leftFLatStatus->setText("Lateral Force:  " + QString::fromStdString(leftFrontLatString.str()));

    QLabel *rightFLatStatus = ui->rightFrontLatQlabel;
    rightFLatStatus->clear();
    rightFrontLatString << std::fixed << std::setprecision(2) << 0.0;
    rightFLatStatus->setText("Lateral Force:  " + QString::fromStdString(rightFrontLatString.str()));

    QLabel *leftRLatStatus = ui->leftRearLatQlabel;
    leftRLatStatus->clear();
    leftRearLatString << std::fixed << std::setprecision(2) << 0.0;
    leftRLatStatus->setText("Lateral Force:  " + QString::fromStdString(leftRearLatString.str()));

    QLabel *rightRLatStatus = ui->rightRearLatQlabel;
    rightRLatStatus->clear();
    rightRearLatString << std::fixed << std::setprecision(2) << 0.0;
    rightRLatStatus->setText("Lateral Force:  " + QString::fromStdString(rightRearLatString.str()));
    QPixmap pix(320, 280);
    pix.fill(Qt::transparent);
    QPainter painter(&pix);
    painter.translate(160.0, 140.0);
    painter.rotate(carSim.steeringAngle);
    painter.drawImage(QPoint(-99, -99), QImage("../fig/steering-icon.png"));
    ui->wheelVisLabel->setPixmap(pix);

}

WheelVis::~WheelVis()
{
    killTimer(timerId);
    delete ui;
}

void WheelVis::keyPressEvent(QKeyEvent *event)
{
    std::ostringstream throttlePowerString;
    std::ostringstream brakePowerString;
    std::ostringstream steeringAngleString;

    if(event->key() == Qt::Key_W)
     {
        carSim.throtlePower += 20.0;
        carSim.throtlePower = std::min(carSim.throtlePower, carSim.maxThrotlePower);
        QLabel *thrtStatus = ui->ThrottleLabel;
        thrtStatus->clear();
        throttlePowerString << std::fixed << std::setprecision(2) << (carSim.throtlePower / carSim.maxThrotlePower) * 100.0;
        thrtStatus->setText("Throttle Power:  " + QString::fromStdString(throttlePowerString.str()) + " %");
    }
    else if(event->key() == Qt::Key_S)
    {
        carSim.throtlePower -= 20.0;
        carSim.throtlePower = std::max(carSim.throtlePower, 0.0);
        QLabel *thrtStatus = ui->ThrottleLabel;
        thrtStatus->clear();
        throttlePowerString << std::fixed << std::setprecision(2) << (carSim.throtlePower / carSim.maxThrotlePower) * 100.0;
        thrtStatus->setText("Throttle Power:  " + QString::fromStdString(throttlePowerString.str()) + " %");
    }


    if(event->key() == Qt::Key_Up)
    {
        carSim.brakePower += 20.0;
        carSim.brakePower = std::min(carSim.brakePower, carSim.maxBrakePower);
        QLabel *brkStatus = ui->BrakeLabel;
        brkStatus->clear();
        brakePowerString << std::fixed << std::setprecision(2) << (carSim.brakePower / carSim.maxBrakePower) * 100.0;
        brkStatus->setText("Brake Power:  " + QString::fromStdString(brakePowerString.str()) + " %");
    }
    else if(event->key() == Qt::Key_Down)
    {
        carSim.brakePower -= 20.0;
        carSim.brakePower = std::max(carSim.brakePower, 0.0);
        QLabel *brkStatus = ui->BrakeLabel;
        brkStatus->clear();
        brakePowerString << std::fixed << std::setprecision(2) << (carSim.brakePower / carSim.maxBrakePower) * 100.0;
        brkStatus->setText("Brake Power:  " + QString::fromStdString(brakePowerString.str()) + " %");
    }


    if(event->key() == Qt::Key_Left)
    {
        QPixmap pix(320, 280);
        pix.fill(Qt::transparent);
        QPainter painter(&pix);
        carSim.steeringAngle -= 2.5;

        carSim.steeringAngle = std::max(carSim.steeringAngle, -carSim.maxSteeringAngle);
        QLabel *strAngStatus = ui->SteeringLabel;
        strAngStatus->clear();
        steeringAngleString << std::fixed << std::setprecision(2) << carSim.steeringAngle;
        strAngStatus->setText("Steering Angle:  " + QString::fromStdString(steeringAngleString.str()) + " deg");

        double wdth = ui->wheelVisLabel->width();
        double hght = ui->wheelVisLabel->height();

        QPoint labelCenter = QPoint(wdth / 2., hght / 2.);

        painter.translate(160.0, 140.0);
        painter.rotate(carSim.steeringAngle);
        QPoint newPoint = QPoint(0, 0);
        painter.drawImage(QPoint(-99, -99), QImage("../fig/steering-icon.png"));
        ui->wheelVisLabel->setPixmap(pix);
    }
    else if(event->key() == Qt::Key_Right)
    {
        QPixmap pix(320, 280);
        pix.fill(Qt::transparent);
        QPainter painter(&pix);
        carSim.steeringAngle += 2.5;

        carSim.steeringAngle = std::min(carSim.steeringAngle, carSim.maxSteeringAngle);
        QLabel *strAngStatus = ui->SteeringLabel;
        strAngStatus->clear();
        steeringAngleString << std::fixed << std::setprecision(2) << carSim.steeringAngle;
        strAngStatus->setText("Steering Angle:  " + QString::fromStdString(steeringAngleString.str()) + " deg");

        painter.translate(160.0, 140.0);
        painter.rotate(carSim.steeringAngle);
        painter.drawImage(QPoint(-99, -99), QImage("../fig/steering-icon.png"));
        ui->wheelVisLabel->setPixmap(pix);
    }
}

void WheelVis::timerEvent(QTimerEvent *event)
{

    double totalForce = carSim.throtlePower - carSim.brakePower - carSim.fR;

    carSim.acceleration += ( totalForce / carSim.mass);
    if (carSim.acceleration >= carSim.Accmax)
    {
        carSim.acceleration = carSim.Accmax;
    }
    else if (carSim.acceleration <= 0.00)
    {
        carSim.acceleration = 0.00;
    }

    double velocity = carSim.acceleration * 0.1;


    carSim.longitudnalVelocity = velocity * cos(carSim.steeringAngle * (M_PI / 180.0));
    carSim.lateralVelocity = -velocity * sin(carSim.steeringAngle * (M_PI / 180.0));

    if (abs(carSim.lateralVelocity) < 0.2)
    {
        carSim.lateralVelocity = 0.0;
    }


    double tmpCarX = 0.0;
    double tmpCarY = 0.0;
    double carFz = (carSim.mass * 9.81) / carSim.numberOfWheels; // this will change when roll and suspension is integrated

    carSim.yawRate += carSim.dotYawRate * 0.1;

    std::vector<double> slipVector = carSim.calculate_slip_angle(carSim.longitudnalVelocity, carSim.lateralVelocity, carSim.steeringAngle * (M_PI / 180.0), carSim.yawRate);

    double effective_z = carFz / (wheel_left_front.tirePressure * 1000.0); // https://www.slideshare.net/PankajDas19/tire-forces-and-moments
    double effective_radius_load = wheel_left_front.tireRadius * (1 - (effective_z / (3 * wheel_left_front.tireRadius))); // https://the-contact-patch.com/book/road/c2019-the-contact-patch
    double longitudnal_slip = (wheel_left_front.tireRadius - effective_radius_load) / wheel_left_front.tireRadius;

    double Fx_lf = wheel_left_front.TempXTyreModel(longitudnal_slip);
    double Fx_rf = wheel_right_front.TempXTyreModel(longitudnal_slip);
    double Fx_lr = wheel_left_rear.TempXTyreModel(longitudnal_slip);
    double Fx_rr = wheel_right_rear.TempXTyreModel(longitudnal_slip);

    double Fy_lf = wheel_left_front.TempYTyreModel(slipVector[0], carFz);
    double Fy_rf = wheel_right_front.TempYTyreModel(slipVector[1], carFz);
    double Fy_lr = wheel_left_rear.TempYTyreModel(slipVector[2], carFz);
    double Fy_rr = wheel_right_rear.TempYTyreModel(slipVector[3], carFz);

    std::vector<double> forcesVector3DLeftFront = {Fx_lf, Fy_lf, carFz};
    wheel_left_front.tyreThermalModel(forcesVector3DLeftFront, carSim.longitudnalVelocity, carSim.lateralVelocity);
    wheel_left_front.updateTimeDelta(0.1);

    std::vector<double> forcesVector3DRightFront = {Fx_rf, Fy_rf, carFz};
    wheel_right_front.tyreThermalModel(forcesVector3DRightFront, carSim.longitudnalVelocity, carSim.lateralVelocity);
    wheel_right_front.updateTimeDelta(0.1);

    std::vector<double> forcesVector3DLeftRear = {Fx_lr, Fy_lr, carFz};
    wheel_left_rear.tyreThermalModel(forcesVector3DLeftRear, carSim.longitudnalVelocity, carSim.lateralVelocity);
    wheel_left_rear.updateTimeDelta(0.1);

    std::vector<double> forcesVector3DRightRear = {Fx_rr, Fy_rr, carFz};
    wheel_right_rear.tyreThermalModel(forcesVector3DRightRear, carSim.longitudnalVelocity, carSim.lateralVelocity);
    wheel_right_rear.updateTimeDelta(0.1);

    carSim.carFx = (Fx_lf + Fx_rf + Fx_lr + Fx_rr) / 4.0;
    carSim.carFy = (Fy_lf + Fy_rf + Fy_lr + Fy_rr) / 4.0;

    carSim.dotYawRate = ((carSim.lf * (Fx_lf + Fx_rf) * sin(carSim.steeringAngle * (M_PI / 180.0))) + (carSim.lf * (Fy_lf + Fy_rf) * cos(carSim.steeringAngle * (M_PI / 180.0)))
                         - (carSim.lr * (Fy_lr + Fy_rr)) + carSim.Mz) / carSim.yawMoment;

    QLabel *lStatus = ui->AccelerationLabel;
    lStatus->clear();
    lStatus->setText("Acceleration:  " + QString::fromStdString(std::to_string(3.6 * carSim.acceleration)));


    std::ostringstream longitudnalString;
    QLabel *fXStatus = ui->LongitudnalLabel;
    fXStatus->clear();
    longitudnalString << std::fixed << std::setprecision(2) << carSim.carFx;
    fXStatus->setText("Longitudnal Force:  " + QString::fromStdString(longitudnalString.str()) + " N");

    std::ostringstream lateralString;
    QLabel *fYStatus = ui->LateralLabel;
    fYStatus->clear();
    lateralString << std::fixed << std::setprecision(2) << carSim.carFy;
    fYStatus->setText("Lateral Force:  " + QString::fromStdString(lateralString.str()) + " N");


    std::ostringstream leftFrontSlipString;
    std::ostringstream leftRearSlipString;
    std::ostringstream rightFrontSlipString;
    std::ostringstream rightRearSlipString;
    QLabel *leftFSlipStatus = ui->leftFrontSlipQlabel;
    leftFSlipStatus->clear();
    leftFrontSlipString << std::fixed << std::setprecision(2) << slipVector[0] * (180.0 / M_PI);
    leftFSlipStatus->setText("Slip Angle:  " + QString::fromStdString(leftFrontSlipString.str()));

    QLabel *leftRSlipStatus = ui->leftRearSlipQlabel;
    leftRSlipStatus->clear();
    leftRearSlipString << std::fixed << std::setprecision(2) << slipVector[2] * (180.0 / M_PI);
    leftRSlipStatus->setText("Slip Angle:  " + QString::fromStdString(leftRearSlipString.str()));

    QLabel *rightFSlipStatus = ui->rightFrontSlipQlabel;
    rightFSlipStatus->clear();
    rightFrontSlipString << std::fixed << std::setprecision(2) << slipVector[1] * (180.0 / M_PI);
    rightFSlipStatus->setText("Slip Angle:  " + QString::fromStdString(rightFrontSlipString.str()));

    QLabel *rightRSlipStatus = ui->rightRearSlipQlabel;
    rightRSlipStatus->clear();
    rightRearSlipString << std::fixed << std::setprecision(2) << slipVector[3] * (180.0 / M_PI);
    rightRSlipStatus->setText("Slip Angle:  " + QString::fromStdString(rightRearSlipString.str()));

    std::ostringstream leftFrontTempString;
    std::ostringstream leftRearTempString;
    std::ostringstream rightFrontTempString;
    std::ostringstream rightRearTempString;


    std::ostringstream leftFrontLongString;
    std::ostringstream leftRearLongString;
    std::ostringstream rightFrontLongString;
    std::ostringstream rightRearLongString;

    std::ostringstream leftFrontLatString;
    std::ostringstream leftRearLatString;
    std::ostringstream rightFrontLatString;
    std::ostringstream rightRearLatString;

    QLabel *leftFLongStatus = ui->leftFrontLongQlabel;
    leftFLongStatus->clear();
    leftFrontLongString << std::fixed << std::setprecision(2) << Fx_lf;
    leftFLongStatus->setText("Longitudnal Force:  " + QString::fromStdString(leftFrontLongString.str()));

    QLabel *rightFLongStatus = ui->rightFrontLongQlabel;
    rightFLongStatus->clear();
    rightFrontLongString << std::fixed << std::setprecision(2) << Fx_rf;
    rightFLongStatus->setText("Longitudnal Force:  " + QString::fromStdString(rightFrontLongString.str()));

    QLabel *leftRLongStatus = ui->leftRearLongQlabel;
    leftRLongStatus->clear();
    leftRearLongString << std::fixed << std::setprecision(2) << Fx_lr;
    leftRLongStatus->setText("Longitudnal Force:  " + QString::fromStdString(leftRearLongString.str()));

    QLabel *rightRLongStatus = ui->rightRearLongQlabel;
    rightRLongStatus->clear();
    rightRearLongString << std::fixed << std::setprecision(2) << Fx_rr;
    rightRLongStatus->setText("Longitudnal Force:  " + QString::fromStdString(rightRearLongString.str()));



    QLabel *leftFLatStatus = ui->leftFrontLatQlabel;
    leftFLatStatus->clear();
    leftFrontLatString << std::fixed << std::setprecision(2) << Fy_lf;
    leftFLatStatus->setText("Lateral Force:  " + QString::fromStdString(leftFrontLatString.str()));

    QLabel *rightFLatStatus = ui->rightFrontLatQlabel;
    rightFLatStatus->clear();
    rightFrontLatString << std::fixed << std::setprecision(2) << Fy_rf;
    rightFLatStatus->setText("Lateral Force:  " + QString::fromStdString(rightFrontLatString.str()));

    QLabel *leftRLatStatus = ui->leftRearLatQlabel;
    leftRLatStatus->clear();
    leftRearLatString << std::fixed << std::setprecision(2) << Fy_lr;
    leftRLatStatus->setText("Lateral Force:  " + QString::fromStdString(leftRearLatString.str()));

    QLabel *rightRLatStatus = ui->rightRearLatQlabel;
    rightRLatStatus->clear();
    rightRearLatString << std::fixed << std::setprecision(2) << Fy_rr;
    rightRLatStatus->setText("Lateral Force:  " + QString::fromStdString(rightRearLatString.str()));


    QLabel *leftFTempStatus = ui->leftFrontTempQlabel;
    leftFTempStatus->clear();
    leftFrontTempString << std::fixed << std::setprecision(2) << wheel_left_front.temperature;
    leftFTempStatus->setText("Temperature:  " + QString::fromStdString(leftFrontTempString.str()));

    QLabel *rightFTempStatus = ui->rightFrontTempQlabel;
    rightFTempStatus->clear();
    rightFrontTempString << std::fixed << std::setprecision(2) << wheel_right_front.temperature;
    rightFTempStatus->setText("Temperature:  " + QString::fromStdString(rightFrontTempString.str()));

    QLabel *leftRTempStatus = ui->leftRearTempQlabel;
    leftRTempStatus->clear();
    leftRearTempString << std::fixed << std::setprecision(2) << wheel_left_rear.temperature;
    leftRTempStatus->setText("Temperature:  " + QString::fromStdString(leftRearTempString.str()));

    QLabel *rightRTempStatus = ui->rightRearTempQlabel;
    rightRTempStatus->clear();
    rightRearTempString << std::fixed << std::setprecision(2) << wheel_right_rear.temperature;
    rightRTempStatus->setText("Temperature:  " + QString::fromStdString(rightRearTempString.str()));
}
