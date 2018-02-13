/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Dreadbot VIII - FIRST Power Up
//Authors: Christian Vaughan, Ethan Leonello, Clara Gauthier, Anna Robelen

#include <iostream>
#include <string>

#include <time.h>
#include "Timer.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <WPILib.h>
#include "ctre/Phoenix.h"

#include "AHRS.h"

class Robot : public frc::IterativeRobot
{
	Compressor *grabComp = new Compressor(0);
	DoubleSolenoid *gSol1 = new DoubleSolenoid(0, 1);
	DoubleSolenoid *gSol2 = new DoubleSolenoid(2, 3);
	WPI_TalonSRX *lf = new WPI_TalonSRX(0); //left front
	WPI_TalonSRX *rf = new WPI_TalonSRX(1); //right front
	WPI_TalonSRX *lr = new WPI_TalonSRX(2); //left rear
	WPI_TalonSRX *rr = new WPI_TalonSRX(3); //right rear
	WPI_TalonSRX *sLift = new WPI_TalonSRX(4); //Skylift
	WPI_TalonSRX *cWinch = new WPI_TalonSRX(5); //climb winch
	WPI_TalonSRX *cExtend = new WPI_TalonSRX(6); //climb extend
	WPI_TalonSRX *pWheelL = new WPI_TalonSRX(7); //pickup wheels left
	WPI_TalonSRX *pWheelR = new WPI_TalonSRX(8); //pickup wheels right

	Ultrasonic *ultra = new Ultrasonic(0, 1); //ultra sonic sensor
	double distance = 0;
	double driveSpeed = 0;
	int iJoystickX_ = 0; // Forward motion
	int iJoystickY_ = 1; // Side motion
	int iJoystickRotate_ = 2; // Rotating motion


	double centerX = 0.0;
	double centerY = 0.0;
	double area = 0.0;

public:

	Joystick *js1;
	Joystick *js2;
	MecanumDrive *m_robotDrive;
    AHRS *gyro;

	void RobotInit()
	{
		//m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		//m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		gyro = new AHRS(SPI::Port::kMXP);

		gyro->ZeroYaw();
		ultra->SetAutomaticMode(true);

    	js1 = new Joystick(0);
    	js2 = new Joystick(1);
    	m_robotDrive = new MecanumDrive(*lf,*lr, *rf,*rr);
    	m_robotDrive->SetExpiration(0.5);
    	m_robotDrive->SetSafetyEnabled(false);

	}


	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */

	int targetAngle = 0;
	double currentAngle = 0;
	double angleRemain = targetAngle - currentAngle;
	double angleSlop = 3;
	bool increaseMaxSpeed = 8;
	int autonPosition = 0;
	bool autonIsLeftSwitch = false;
	bool autonIsBlueAlliance = false;

	void Wait(double t)
	{
		clock_t wait;
		wait = clock();
		double start = wait;
		double timeLeft = (start + t) - wait;
		while (wait <= start + t)
		{
			wait = clock();
			distance = ultra->GetRangeInches();
			frc::SmartDashboard::PutNumber("distance", distance);
			timeLeft = (start + t) - wait;
			frc::SmartDashboard::PutNumber("Time Left", timeLeft);
		}

		return;
	}

	void DriveForward(double x, double t)
	{
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

	void DriveSideways(double x, double t)
	{
		lf -> Set(ControlMode::PercentOutput, -x);
		lr -> Set(ControlMode::PercentOutput, x);
		rf -> Set(ControlMode::PercentOutput, -x);
		rr -> Set(ControlMode::PercentOutput, x);
		Wait(t);
		lf -> Set(ControlMode::PercentOutput, 0);
		lr -> Set(ControlMode::PercentOutput, 0);
		rf -> Set(ControlMode::PercentOutput, 0);
		rr -> Set(ControlMode::PercentOutput, 0);
	}

	void Lifter(int x)
	{
		//raise or lower the lifting device
		if (x == 0)
		{

		}

		else if(x == 1)
		{

		}

		else
		{

		}
	}

	void Drop()
	{
		//release the cube
	}

	void TurnToAngle(int targetAngle)
	{
			//Turn setting motor speed to  currentAngle / targetAngle and have a minimum of .1
		currentAngle = gyro->GetYaw();
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
	void DriveToBlock(){
	double distance = ultra->GetRangeInches();
	SmartDashboard::PutNumber("DriveSpeed", driveSpeed);
	//if (driveMode != DriveMode::Driving){
	//return;
	//	}
			if (distance >= 44){
			lf ->Set(ControlMode::PercentOutput, -driveSpeed);
			lr ->Set(ControlMode::PercentOutput, -driveSpeed);
			rf ->Set(ControlMode::PercentOutput, driveSpeed);
			rr ->Set(ControlMode::PercentOutput, driveSpeed);

			distance = ultra->GetRangeInches();
			SmartDashboard::PutNumber("Distance", distance);
				}
		if (distance < 44 && distance >= 8){

		driveSpeed = driveSpeed * 0.9;

		}
		if (distance < 8) {
		//driveMode = DriveMode::Pickup;
		lf ->Set(ControlMode::PercentOutput, 0);
		lr ->Set(ControlMode::PercentOutput, 0);
		rf ->Set(ControlMode::PercentOutput, 0);
	    rr ->Set(ControlMode::PercentOutput, 0);
		}
	//here's where we need to grab the block
	}

	//This extends and retracts the pole. It also sets the motor to 0
	void ActuateBirdPole(int extend)
		{
			if (extend > 0)
			{
				cExtend->Set(0.5);
			}

			else if (extend < 0)
			{
				cExtend->Set(-0.5);
			}

			else if (extend == 0)
			{
				cExtend->Set(0.0);
			}
		}

	void teleopSkyLift()
	{
		bool goingUp = js1->GetRawButton(7);
		bool goingDown = js1->GetRawButton(5);

		if(!goingUp && !goingDown){
			sLift ->Set(ControlMode::PercentOutput, 0.75);
		}
		else if(goingUp){
		 sLift ->Set(ControlMode::PercentOutput, 1);
		}
		else if(goingDown){
			sLift ->Set(ControlMode::PercentOutput, 0);
		}
	}
	void teleopArmControl(){
		if(js2->GetRawButton(5)){
			gSol1->Set(DoubleSolenoid::Value::kReverse);
		}
		if(js2->GetRawButton(7)){
			gSol1->Set(DoubleSolenoid::Value::kForward);
		}
		if(js2->GetRawButton(4)){
			gSol2->Set(DoubleSolenoid::Value::kReverse);
		}
		if(js2->GetRawButton(1)){
			gSol2->Set(DoubleSolenoid::Value::kForward);
		}
	}
	void pickUpWheels(){
		if(js2->GetRawButton(8)){
			pWheelL->Set(ControlMode::PercentOutput, .5);
			pWheelR->Set(ControlMode::PercentOutput, -.5);
		}
		else if (js2->GetRawButton(6)){
			pWheelL->Set(ControlMode::PercentOutput, .5);
			pWheelR->Set(ControlMode::PercentOutput, -.5);

		}
		else{
			pWheelL->Set(ControlMode::PercentOutput, 0);
			pWheelR->Set(ControlMode::PercentOutput, 0);
		}



	}

	void LeftOne()
	{
		DriveForward(1, 1000);
		TurnToAngle(-90);
		Lifter(1);
		DriveForward(1, 1000);
		TurnToAngle(-90);
		Drop();
	}


	void LeftTwo()
	{
		Wait(5000);
		DriveSideways(-1, 1000);
		Lifter(1);
		DriveForward(1, 1000);
		Drop();
	}

	void LeftThree()
	{
		DriveForward(1, 1000);
		Lifter(1);
		TurnToAngle(90);
		DriveForward(1, 1000);
		Drop();
	}

	void RightOne()
	{
		DriveForward(1, 1000);
		Lifter(1);
		TurnToAngle(-90);
		DriveForward(1, 1000);
		Drop();
	}

	void RightTwo()
	{
		Wait(5);
		DriveSideways(1, 1000);
		Lifter(1);
		DriveForward(1, 1000);
		Drop();
	}

	void RightThree()
	{
		DriveForward(1, 1000);
		TurnToAngle(90);
		Lifter(1);
		DriveForward(1, 1000);
		TurnToAngle(90);
		Drop();
	}

	void Drive(double xAxis, double yAxis, double rot)
	{
		double noMove = 0.2; //Dead area of the axes
		double maxSpeed = .5;

		if (fabs(xAxis) < noMove)
			xAxis = 0.0;

		if (fabs(yAxis) < noMove)
			yAxis = 0.0;

		if (fabs(rot) < noMove)
			rot = 0.0;

		if (js1->GetRawButton(increaseMaxSpeed))
			maxSpeed = 1;

		else
			maxSpeed = .5;

		double lfSpeed = -xAxis + yAxis + rot;
		double lrSpeed = xAxis + yAxis + rot;
		double rfSpeed = -xAxis + yAxis - rot;
		double rrSpeed = xAxis + yAxis - rot;

		if (fabs(lfSpeed) > 1)
			lfSpeed = fabs(lfSpeed) / lfSpeed;

		if (fabs(lrSpeed) > 1)
			lrSpeed = fabs(lrSpeed) / lrSpeed;

		if (fabs(rfSpeed) > 1)
			rfSpeed = fabs(rfSpeed) / rfSpeed;

		if (fabs(rrSpeed) > 1)
			rrSpeed = fabs(rrSpeed) / rrSpeed;

		lf -> Set(ControlMode::PercentOutput, lfSpeed*maxSpeed);
		lr -> Set(ControlMode::PercentOutput, lrSpeed*maxSpeed);
		rf -> Set(ControlMode::PercentOutput, rfSpeed*maxSpeed);
		rr -> Set(ControlMode::PercentOutput, rrSpeed*maxSpeed);
	}

	char givenPos = ' '; //Where we start against the alliance wall
	char switchRL = ' '; //Where our side if the switch is

	void AutonomousInit() override
	{
		//SmartDashboard::PutNumber("Auton Position", autonPosition);
		//SmartDashboard::PutBoolean("Auton Is Blue Alliance?", autonIsBlueAlliance);
		//SmartDashboard::PutBoolean("Auton Is Going to Left Switch?", autonIsLeftSwitch);

		SmartDashboard::GetNumber("Auton Position", autonPosition);
		SmartDashboard::GetBoolean("Auton Is Blue Alliance", autonIsBlueAlliance);

		if (autonIsBlueAlliance == true)
		{
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
		}

		else if(autonIsBlueAlliance == false)
		{
//			if(/*left switch is red*/)
//			{
//				autonIsLeftSwitch = true;
//			}
		}

		SmartDashboard::GetBoolean("Auton Is Going to Left Switch?", autonIsLeftSwitch);

		currentAngle = 0;
		gyro->ZeroYaw();
		}

	void AutonomousPeriodic()
	{
		//These are the numbers coming from the vision camera
		centerX = SmartDashboard::GetNumber("yellowbox.contour_1.cx", -1);
		centerY = SmartDashboard::GetNumber("yellowbox.contour_1.cy", -1);
		area = SmartDashboard::GetNumber("yellowbox.contour_1.area", -1);

		//Putting the same values as X and Y values for easy identification
		SmartDashboard::PutNumber("Center X ", centerX);
		SmartDashboard::PutNumber("Center Y ", centerY);
		SmartDashboard::PutNumber("Area ", area);

		distance = ultra->GetRangeInches();
		frc::SmartDashboard::PutNumber("distance", distance);

		currentAngle = gyro->GetYaw();
		frc::SmartDashboard::PutNumber("Current Angle", currentAngle);

		SmartDashboard::GetNumber("Auton Position", autonPosition);
		SmartDashboard::GetBoolean("Auton Is Blue Alliance", autonIsBlueAlliance);
		SmartDashboard::GetBoolean("Auton Is Going to Left Switch?", autonIsLeftSwitch);

		if(autonPosition == 1)
		{
			if (autonIsLeftSwitch == true)
				LeftOne();

			else
				RightOne();
		}

		if(autonPosition == 2)
		{
			if (autonIsLeftSwitch == true)
				LeftTwo();

			else
				RightTwo();
		}

		if(autonPosition == 3)
		{
			if (autonIsLeftSwitch == true)
				LeftThree();

			else
				RightThree();
		}

	}

    double Db(double axisVal)
    {
    	if(axisVal < -0.10)
    		return axisVal;
    	if(axisVal > 0.10)
    		return axisVal;
    	return 0;
    }

	void TeleopInit()
	{
		gyro->Reset();
	}

	void TeleopPeriodic()
	{
		float angle = gyro->GetAngle();
		if (js1->GetRawButton(8)) {
			m_robotDrive->DriveCartesian(Db(js1->GetY()), Db(-js1->GetX()), Db(-js1->GetZ()), angle);
		}
		else{
			m_robotDrive->DriveCartesian(0.5*Db(js1->GetY()), 0.5*Db(-js1->GetX()), 0.5*Db(-js1->GetZ()), angle);
		}
		if(js1->GetRawButton(5))
				gyro->Reset();

		teleopArmControl();
		pickUpWheels();

		int PoleExtend=0;
		if (js1->GetRawButton(3)){ //Button B extends the pole
			PoleExtend=1;
		}
		if (js1->GetRawButton(2)){ //Button A retracts the pole
			PoleExtend=-1;
		}

		ActuateBirdPole(PoleExtend);

	}

	void TestPeriodic()
	{

	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	//const std::string kAutoNameDefault = "Default";
	//const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
