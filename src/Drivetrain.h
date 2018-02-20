/*
 * Encoder.h
 *
 *  Created on: Feb 15, 2018
 *      Author: Justin, etc.
 */

#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <math.h>
#include <ultrasonic.h>
#include <AHRS.h>

#define CIRCU 23.5619449019

class Drivetrain {

public:
	double angleSlop = 3;
	double rotationNumb=0.0;
	int loopidx;
	int toums;
	double tolerance = 400;
	double P;
	double I;
	double D;
	WPI_TalonSRX *lf; /*left front */
	WPI_TalonSRX *rf; /*right front */
	WPI_TalonSRX *lr;/*left rear */
	WPI_TalonSRX *rr; /*right rear */
	WPI_TalonSRX *TOP; //Top Motor
	AHRS *gyro;

	Drivetrain(WPI_TalonSRX *lf_in, WPI_TalonSRX *rf_in, WPI_TalonSRX *lr_in, WPI_TalonSRX *rr_in, WPI_TalonSRX *TOP_in) {

		gyro = new AHRS(SPI::Port::kMXP);

		lf = lf_in;
		rf = rf_in;
		lr = lr_in;
		rr = rr_in;
		TOP =TOP_in;

		//Constants Bellow

		loopidx = 0;
		toums = 10;
		P = 0.1;
		I = 0.0;
		D = 0.0;
		double peakPOWER = 1;

		//Test Robot Garbage Bellow

		int absP = TOP->GetSelectedSensorPosition(0) & 0xFFF;

		TOP->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,10);
		TOP->SetSelectedSensorPosition(absP, 0, 10);
		TOP->SetSensorPhase(true);
		TOP->ConfigNominalOutputForward(0,10);
		TOP->ConfigNominalOutputReverse(0,10);
		TOP->ConfigPeakOutputForward(1,10);
		TOP->ConfigPeakOutputReverse(1,10);

		TOP->Config_kF(0,0.0,10);
		TOP->Config_kP(0,P,10);
		TOP->Config_kI(0,I,10);
		TOP->Config_kD(0,D,10);

		//Actual Motors Bellow

		lf->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,loopidx,toums);
		lr->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,loopidx,toums);
		rf->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,loopidx,toums);
		rr->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,loopidx,toums);

		int lfabsPos = lf->GetSelectedSensorPosition(0) & 0xFFF;
		int lrabsPos = lr->GetSelectedSensorPosition(0) & 0xFFF;
		int rfabsPos = rf->GetSelectedSensorPosition(0) & 0xFFF;
		int rrabsPos = rr->GetSelectedSensorPosition(0) & 0xFFF;

		lf->SetSelectedSensorPosition(lfabsPos,loopidx,toums);
		lr->SetSelectedSensorPosition(lrabsPos,loopidx,toums);
		rf->SetSelectedSensorPosition(rfabsPos,loopidx,toums);
		rr->SetSelectedSensorPosition(rrabsPos,loopidx,toums);

		lf->SetSensorPhase(false);
		lr->SetSensorPhase(false);
		rf->SetSensorPhase(true);
		rr->SetSensorPhase(true);

		//Invert Right Side (Maybe, Might not be needed)
		//		rf->SetInverted(true);
		//		rr->SetInverted(true);

		lf->ConfigNominalOutputForward(0,toums);
		lr->ConfigNominalOutputForward(0,toums);
		rf->ConfigNominalOutputForward(0,toums);
		rr->ConfigNominalOutputForward(0,toums);

		lf->ConfigNominalOutputReverse(0,toums);
		lr->ConfigNominalOutputReverse(0,toums);
		rf->ConfigNominalOutputReverse(0,toums);
		rr->ConfigNominalOutputReverse(0,toums);

		lf->ConfigPeakOutputForward(peakPOWER,toums);
		lr->ConfigPeakOutputForward(peakPOWER,toums);
		rf->ConfigPeakOutputForward(peakPOWER,toums);
		rr->ConfigPeakOutputForward(peakPOWER,toums);

		lf->ConfigPeakOutputReverse(peakPOWER,toums);
		lr->ConfigPeakOutputReverse(peakPOWER,toums);
		rf->ConfigPeakOutputReverse(peakPOWER,toums);
		rr->ConfigPeakOutputReverse(peakPOWER,toums);

		lf->Config_kF(loopidx,0.0,toums);
		lr->Config_kF(loopidx,0.0,toums);
		rf->Config_kF(loopidx,0.0,toums);
		rr->Config_kF(loopidx,0.0,toums);

		lf->Config_kP(loopidx,P,toums);
		lr->Config_kP(loopidx,P,toums);
		rf->Config_kP(loopidx,P,toums);
		rr->Config_kP(loopidx,P,toums);

		lf->Config_kI(loopidx,I,toums);
		lr->Config_kI(loopidx,I,toums);
		rf->Config_kI(loopidx,I,toums);
		rr->Config_kI(loopidx,I,toums);

		lf->Config_kD(loopidx,D,toums);
		lr->Config_kD(loopidx,D,toums);
		rf->Config_kD(loopidx,D,toums);
		rr->Config_kD(loopidx,D,toums);

	}

	~Drivetrain(){

	}

	void DrivePosition(double d){
		lf->SetSelectedSensorPosition(0, 0, 10);
		rf->SetSelectedSensorPosition(0, 0, 10);
		lr->SetSelectedSensorPosition(0, 0, 10);
		rf->SetSelectedSensorPosition(0, 0, 10);
		rotationNumb = (CIRCU * 4096)/d;
		lf->Set(ControlMode::Position,rotationNumb);
		rf->Set(ControlMode::Position,rotationNumb);
		lr->Set(ControlMode::Position,rotationNumb);
		rr->Set(ControlMode::Position,rotationNumb);
	}

	void setPID(){
		P= frc::SmartDashboard::GetNumber("P",0.1);
		I=frc::SmartDashboard::GetNumber("I",0.0);;
		D=frc::SmartDashboard::GetNumber("D",0.0);;
		lf->Config_kP(loopidx,P,toums);
		lr->Config_kP(loopidx,P,toums);
		rf->Config_kP(loopidx,P,toums);
		rr->Config_kP(loopidx,P,toums);

		lf->Config_kI(loopidx,I,toums);
		lr->Config_kI(loopidx,I,toums);
		rf->Config_kI(loopidx,I,toums);
		rr->Config_kI(loopidx,I,toums);

		lf->Config_kD(loopidx,D,toums);
		lr->Config_kD(loopidx,D,toums);
		rf->Config_kD(loopidx,D,toums);
		rr->Config_kD(loopidx,D,toums);
	}

	bool isdoneStraight(){
		double a = 0.0;
		a+=lf->GetClosedLoopError(rotationNumb);
		a+=rf->GetClosedLoopError(rotationNumb);
		a+=lr->GetClosedLoopError(rotationNumb);
		a+=rr->GetClosedLoopError(rotationNumb);
		if(a/4<tolerance){
			return true;
		}
		return false;
	}
	void MechDrive(double joystick_x,double joystick_y,double joystick_z)
	{

	}
	void TurboBOOSTO (bool booly){

	}

	void TankArcade (double x, double y){

	}
	void TurnToAngle(double targetAngle){

		double currentAngle = gyro->GetYaw();
				double x = targetAngle / fabs(currentAngle);

				while (fabs(currentAngle) < fabs(targetAngle) - angleSlop)
				{
					if (x > 1)
						x = 1;
					lf -> Set(ControlMode::PercentOutput, x);
					lr -> Set(ControlMode::PercentOutput, x);
					rf -> Set(ControlMode::PercentOutput, -x);
					rr -> Set(ControlMode::PercentOutput, -x);
					currentAngle = gyro->GetYaw();
					x = targetAngle / currentAngle;

					}

	}


	void STOPYCUBEY(){

	}

	void PercentDriveForward(double x, double t){
		lf -> Set(ControlMode::PercentOutput, x);
		lr -> Set(ControlMode::PercentOutput, x);
		rf -> Set(ControlMode::PercentOutput, x);
		rr -> Set(ControlMode::PercentOutput, x);
		Wait(t); //Motors set for t time
		lf -> Set(ControlMode::PercentOutput, 0);
		lr -> Set(ControlMode::PercentOutput, 0);
		rf -> Set(ControlMode::PercentOutput, 0);
		rr -> Set(ControlMode::PercentOutput, 0);
		x = 0;
		t = 0;
	}


};


