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


#define ipr 10.20945189011087

class Drivetrain {

public:
	double angleSlop = 3;
	double rotationNumb=0.0;
	int loopidx;
	int toums;
	double tolerance = 2048;

	double P;
	double I;
	double D;
	double Pl;
	double Il;
	double Dl;
	WPI_TalonSRX *lf; /*left front */
	WPI_TalonSRX *rf; /*right front */
	WPI_TalonSRX *lr;/*left rear */
	WPI_TalonSRX *rr; /*right rear */
	WPI_TalonSRX *sLift; //lifty motor
	AHRS *gyro;


	Drivetrain(WPI_TalonSRX *lf_in, WPI_TalonSRX *rf_in, WPI_TalonSRX *lr_in, WPI_TalonSRX *rr_in, WPI_TalonSRX *sLift_in) {

		gyro = new AHRS(SPI::Port::kMXP);

		lf = lf_in;
		rf = rf_in;
		lr = lr_in;
		rr = rr_in;
		sLift =sLift_in;

		//Constants Bellow

		loopidx = 0;
		toums = 10;

		P = 0.1;
		I = 0.05;
		D = 0.0;
		Pl = 0.1;
		Il= 0.0;
		Dl = 0.0;
		double peakPOWER = .75;

		//Test Robot Garbage Bellow

		int absP = sLift->GetSelectedSensorPosition(0) & 0xFFF;

		sLift->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,10);
		sLift->SetSelectedSensorPosition(absP, 0, 10);
		sLift->SetSensorPhase(true);
		sLift->ConfigNominalOutputForward(0,toums);
		sLift->ConfigNominalOutputReverse(0,toums);
		sLift->ConfigPeakOutputForward(0.8,toums);
		sLift->ConfigPeakOutputReverse(0.8,toums);

		sLift->Config_kF(0,0.0,10);
		sLift->Config_kP(0,Pl,10);
		sLift->Config_kI(0,Il,10);
		sLift->Config_kD(0,Dl,10);

		//Actual Motors Bellow

		//lf->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,loopidx,toums);
		lr->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,loopidx,toums);
		//rf->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,loopidx,toums);
		//rr->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,loopidx,toums);

		//int lfabsPos = lf->GetSelectedSensorPosition(0) & 0xFFF;
		int lrabsPos = lr->GetSelectedSensorPosition(0) & 0xFFF;
		//int rfabsPos = rf->GetSelectedSensorPosition(0) & 0xFFF;
		//int rrabsPos = rr->GetSelectedSensorPosition(0) & 0xFFF;

		//lf->SetSelectedSensorPosition(lfabsPos,loopidx,toums);
		lr->SetSelectedSensorPosition(lrabsPos,loopidx,toums);
		//rf->SetSelectedSensorPosition(rfabsPos,loopidx,toums);
		//rr->SetSelectedSensorPosition(rrabsPos,loopidx,toums);

		lf->SetSensorPhase(true);
		lr->SetSensorPhase(true);
		rf->SetSensorPhase(false);
		rr->SetSensorPhase(false);

		lf->Follow(*lr);
		rf->Follow(*lr);
		rr->Follow(*lr);

		//Invert Right Side (Maybe, Might not be needed)
		//		rf->SetInverted(true);
		//		rr->SetInverted(true);

		//lf->ConfigNominalOutputForward(0,toums);
		lr->ConfigNominalOutputForward(0,toums);
		//rf->ConfigNominalOutputForward(0,toums);
		//rr->ConfigNominalOutputForward(0,toums);

		//lf->ConfigNominalOutputReverse(0,toums);
		lr->ConfigNominalOutputReverse(0,toums);
		//rf->ConfigNominalOutputReverse(0,toums);
		//rr->ConfigNominalOutputReverse(0,toums);

		//lf->ConfigPeakOutputForward(peakPOWER,toums);
		lr->ConfigPeakOutputForward(peakPOWER,toums);
		//rf->ConfigPeakOutputForward(peakPOWER,toums);
		//rr->ConfigPeakOutputForward(peakPOWER,toums);

		//lf->ConfigPeakOutputReverse(peakPOWER,toums);
		lr->ConfigPeakOutputReverse(peakPOWER,toums);
		//rf->ConfigPeakOutputReverse(peakPOWER,toums);
		//rr->ConfigPeakOutputReverse(peakPOWER,toums);

		//lf->Config_kF(loopidx,0.0,toums);
		lr->Config_kF(loopidx,0.0,toums);
		//rf->Config_kF(loopidx,0.0,toums);
		//rr->Config_kF(loopidx,0.0,toums);

		//lf->Config_kP(loopidx,P,toums);
		lr->Config_kP(loopidx,P,toums);
		//rf->Config_kP(loopidx,P,toums);
		//rr->Config_kP(loopidx,P,toums);

		//lf->Config_kI(loopidx,I,toums);
		lr->Config_kI(loopidx,I,toums);
		//rf->Config_kI(loopidx,I,toums);
		//rr->Config_kI(loopidx,I,toums);

		//lf->Config_kD(loopidx,D,toums);
		lr->Config_kD(loopidx,D,toums);
		//rf->Config_kD(loopidx,D,toums);
		//rr->Config_kD(loopidx,D,toums);



	}

	~Drivetrain(){

	}

	void DrivePosition(double d){
		lf->SetSelectedSensorPosition(0, 0, 10);
		rf->SetSelectedSensorPosition(0, 0, 10);
		lr->SetSelectedSensorPosition(0, 0, 10);
		rf->SetSelectedSensorPosition(0, 0, 10);
		rotationNumb = (d * ipr) * 4096;
		//lf->Set(ControlMode::Position,rotationNumb);
		//rf->Set(ControlMode::Position,rotationNumb);
		lr->Set(ControlMode::Position,rotationNumb);
		//rr->Set(ControlMode::Position,rotationNumb);
	}

	void setPID(){
		P= frc::SmartDashboard::GetNumber("P",0.1);
		I=frc::SmartDashboard::GetNumber("I",0.0);;
		D=frc::SmartDashboard::GetNumber("D",0.0);;
		//lf->Config_kP(loopidx,P,toums);
		lr->Config_kP(loopidx,P,toums);
		//rf->Config_kP(loopidx,P,toums);
		//rr->Config_kP(loopidx,P,toums);

		//lf->Config_kI(loopidx,I,toums);
		lr->Config_kI(loopidx,I,toums);
		//rf->Config_kI(loopidx,I,toums);
		//rr->Config_kI(loopidx,I,toums);

		//lf->Config_kD(loopidx,D,toums);
		lr->Config_kD(loopidx,D,toums);
		//rf->Config_kD(loopidx,D,toums);
		//rr->Config_kD(loopidx,D,toums);
	}

	bool isdoneStraight(){
		double a = 0.0;
		//a+=lf->GetClosedLoopError(rotationNumb);
		//a+=rf->GetClosedLoopError(rotationNumb);
		a+=lr->GetClosedLoopError(rotationNumb);
		//a+=rr->GetClosedLoopError(rotationNumb);
		if(-tolerance<a&&a<tolerance){
			return true;
		}
		SmartDashboard::PutNumber("Error for the thing",lr->GetClosedLoopError(rotationNumb));
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


	void STOPEY(){

	}
	void Wait(double t)
		{
			int wait = 0;
			double timeLeft = t - wait;
			while (wait <= t)
			{
				timeLeft = t - wait;
				frc::SmartDashboard::PutNumber("Time Left", timeLeft);
				wait++;
			}

			return;
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


private:

};
