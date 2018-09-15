/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Dreadbot VIII - FIRST Power Up
//Authors: Christian Vaughan, Ethan Leonello, Clara Gauthier, Anna Robelen, Robert Lindskov, Justin McIntosh, Kyle Roleson, Grant Reamy

#include <iostream>
#include <string>

#include <time.h>
#include "Timer.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <WPILib.h>
#include <ctre/Phoenix.h>

#include <AHRS.h>
class Robot : public frc::IterativeRobot
{
//a
//------------------------------------------------------
//Solenoid Variables
//------------------------------------------------------
	Compressor *grabComp = new Compressor(0);
	Solenoid *armSol = new Solenoid(0); //arms up and down
	Solenoid *clawSol = new Solenoid(1); //open claw
	Solenoid *birdSol = new Solenoid(2); //deploy bird pole

//------------------------------------------------------
//Talon Variables
//------------------------------------------------------
	WPI_TalonSRX *lf = new WPI_TalonSRX(0); //left front
	WPI_TalonSRX *rf = new WPI_TalonSRX(1); //right front
	WPI_TalonSRX *lr = new WPI_TalonSRX(2); //left rear
	WPI_TalonSRX *rr = new WPI_TalonSRX(3); //right rear
	WPI_TalonSRX *sLift = new WPI_TalonSRX(4); //Sky lift
	WPI_TalonSRX *cWinch = new WPI_TalonSRX(5); //climb winch
	WPI_TalonSRX *cWinch2 = new WPI_TalonSRX(9);// second climb winch
	WPI_TalonSRX *cExtend = new WPI_TalonSRX(6); //climb extend
	WPI_TalonSRX *pWheelL = new WPI_TalonSRX(7); //pickup wheels left
	WPI_TalonSRX *pWheelR = new WPI_TalonSRX(8); //pickup wheels right


//------------------------------------------------------
//Ultrasonic Variables
//------------------------------------------------------
	Ultrasonic *ultra = new Ultrasonic(0, 1); //ultra sonic sensor
	double distance = 0;
	double driveSpeed = 0;



//------------------------------------------------------
//Controller Variables
//------------------------------------------------------

	//joystick variables
	int joystickX = 0;
	int joystickY=1;
	int climbTheScale = 1;
	int joystickRot=2;

	//controller 1 buttons
	int retractBird = 2;
	int extendBird = 3;
//	int deployBirdArm = 4;
	int skyLiftDown = 5;
	int cubeDrop = 6;
	int skyLiftUp = 7;
	int turboButton = 8;

	//controller 2 buttons
	int closeArms = 1;
	int openArms = 4;
	int pickupArmsUp = 5;
	int throwCube = 8;//orig 6
	int pickupArmsDown = 7;
	int captureCube = 6;//orig 8


//------------------------------------------------------
//Auton Variables
//------------------------------------------------------
//	int targetAngle = 0;
//	double currentAngle = 0;
//	double angleRemain = targetAngle - currentAngle;
//	double angleSlop = 3;
//	int autonPosition = 0;
//	bool autonIsLeftSwitch = false;
//	bool autonIsBlueAlliance = false;
	double robotPos;
	bool encoders;
	double timer;
	int state = 0;
	std::string gameData;
	float currentAngle;
	double autonSpeed = 0.4;
	double autonTurnSpeed = 0.4;
	double FirstAction;
	double SecondAction;
	double ThirdAction;
	double FourthAction;
	double FifthAction;
	double SixthAction;
	double LeftDifference = 0;
	double RightDifference = 0;
	bool LiftComplete;
	frc::SendableChooser<int> posChooser;
	frc::SendableChooser<bool> encodeChooser;
	frc::SendableChooser<bool> halfwayChooser;
	frc::SendableChooser<bool> doubleChooser;
	double oneSecond = 50;
	double halfSecond = 25;
	double twoSeconds = 100;
	double timeCon = 0.75*(50.0/78.0);// The robot drives about 156 inches each second at 0.4 speed.
	double strafeCon = 50.0/31.5;//The robot strafes about 21 inches each second
	double liftCon = 50.0/40.0;//The robot lifts about 40 inches each second
	bool done = false;
	int LRsensorPos;
	int LFsensorPos;
	int RRsensorPos;
	int RFsensorPos;
	int LiftsensorPos;
	double ipr = (1.0/380.0)/1.0;
	double iprLift = 10.21;
	int currentRot;
	int currentLift;
	int liftRot;
	double LiftHeight;
	double FirstDistance;
	double SecondDistance;
	double ThirdDistance;
	double OldThirdDistance;
	double FourthDistance;
	double FifthDistance;
	double FirstDistanceV2;
	int firstRot;
	int secondRot;
	int thirdRot;
	int fourthRot;
	int fifthRot;
	int firstRotV2;
	bool halfFarScale;
	bool DoubleScale;

	double blockAngle = 0;
	const double FocalLength = 424;
	const double Pi = 3.1415;
	const double Offset = 0.0;
	double angleTotal = 0;
	bool foundBlock = false;
	bool State7Done = false;
	double BlockDistance = 0.0;
	int blockRotation = 0;
	double reverseDistance = 0.0;
	int reverseRotation = 0;
	double sixthDistance = 0.0;
	int sixthRotation = 0;
	double seventhDistance = 0.0;

//------------------------------------------------------
//Vision Variables
//------------------------------------------------------
	double centerX = 0.0;
	double centerY = 0.0;
	double area = 0.0;

//------------------------------------------------------
//Drive Variables
//------------------------------------------------------
	double teleMaxSpeed = 0.5;
	bool isBirdOut = false;
	bool isYDown = false;




public:

	Joystick *js1; //primary controller
	Joystick *js2; //secondary controller
	MecanumDrive *m_robotDrive;
    AHRS *gyro;

	void RobotInit()
	{
		CameraServer::GetInstance()->StartAutomaticCapture();
		//m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		//m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		//frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		gyro = new AHRS(SPI::Port::kMXP);

		gyro->ZeroYaw();
		ultra->SetAutomaticMode(true);

    	js1 = new Joystick(0);
    	js2 = new Joystick(1);
    	m_robotDrive = new MecanumDrive(*lf,*lr, *rf,*rr);
    	m_robotDrive->SetExpiration(0.5);
    	m_robotDrive->SetSafetyEnabled(false);
    	//SmartDashboard::PutNumber("robotPos", robotPos);
    	posChooser.AddDefault("Auton Line", 0);
    	posChooser.AddObject("Left", 1);
    	posChooser.AddObject("Center", 2);
    	posChooser.AddObject("Right", 3);
    	encodeChooser.AddDefault("Yes", true);
    	encodeChooser.AddObject("No", false);
    	halfwayChooser.AddDefault("Yes", true);
    	halfwayChooser.AddObject("No", false);
    	doubleChooser.AddObject("Yes", true);
    	doubleChooser.AddObject("No", false);
    	frc::SmartDashboard::PutData("robotPos", &posChooser);
    	frc::SmartDashboard::PutData("Using Encoders?", &encodeChooser);
    	frc::SmartDashboard::PutData("Halfway Scale?", &halfwayChooser);
    	frc::SmartDashboard::PutData("Double Scale?", &doubleChooser);
    	lf->ConfigNominalOutputForward(0,0);
    	lr->ConfigNominalOutputForward(0,0);
    	rf->ConfigNominalOutputForward(0,0);
    	rr->ConfigNominalOutputForward(0,0);
    	sLift->ConfigNominalOutputForward(0,0);

    	lf->ConfigNominalOutputReverse(0,0);
    	lr->ConfigNominalOutputReverse(0,0);
    	rf->ConfigNominalOutputReverse(0,0);
    	rr->ConfigNominalOutputReverse(0,0);
    	sLift->ConfigNominalOutputReverse(0,0);

    	lf->ConfigPeakOutputForward(1,0);
    	lr->ConfigPeakOutputForward(1,0);
    	rf->ConfigPeakOutputForward(1,0);
    	rr->ConfigPeakOutputForward(1,0);
    	sLift->ConfigPeakOutputForward(1,0);

    	lf->ConfigPeakOutputReverse(-1,0);
    	lr->ConfigPeakOutputReverse(-1,0);
    	rf->ConfigPeakOutputReverse(-1,0);
    	rr->ConfigPeakOutputReverse(-1,0);
    	sLift->ConfigPeakOutputReverse(-1,0);


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


 

	void Wait(double t){
		int wait = 0;
		double timeLeft = t - wait;

		while (wait <= t)
		{
			distance = ultra->GetRangeInches();
			frc::SmartDashboard::PutNumber("distance", distance);
			timeLeft = t - wait;
			frc::SmartDashboard::PutNumber("Time Left", timeLeft);
			wait++;
		}

		return;
	}
	void Stop(){
		lf -> Set(ControlMode :: PercentOutput, 0);
		rf -> Set(ControlMode :: PercentOutput, 0);
		lr -> Set(ControlMode :: PercentOutput, 0);
		rr -> Set(ControlMode :: PercentOutput, 0);
	}
	void TurnRight(double speed){
		lf -> Set(ControlMode :: PercentOutput, -speed);
		lr -> Set(ControlMode :: PercentOutput, -speed);
		rf -> Set(ControlMode :: PercentOutput, -speed);
		rr -> Set(ControlMode :: PercentOutput, -speed);
	}
	void TurnLeft(double speed){
		lf -> Set(ControlMode :: PercentOutput, speed);
		lr -> Set(ControlMode :: PercentOutput, speed);
		rf -> Set(ControlMode :: PercentOutput, speed);
		rr -> Set(ControlMode :: PercentOutput, speed);
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
	void DriveStraight(double speed){
		float slop = 3;

		if(currentAngle < 0 - slop){
			lf -> Set(ControlMode :: PercentOutput, -speed);
			rf -> Set(ControlMode :: PercentOutput, speed+LeftDifference);
			lr -> Set(ControlMode :: PercentOutput, -speed);
			rr -> Set(ControlMode :: PercentOutput, speed+LeftDifference);
		}
		else if(currentAngle > 0 + slop){
			lf -> Set(ControlMode :: PercentOutput, -speed+RightDifference);
			rf -> Set(ControlMode :: PercentOutput, speed);
			lr -> Set(ControlMode :: PercentOutput, -speed+RightDifference);
			rr -> Set(ControlMode :: PercentOutput, speed);

		}

		else{
			rf -> Set(ControlMode :: PercentOutput, speed);
			lf -> Set(ControlMode :: PercentOutput, -speed);
			lr -> Set(ControlMode :: PercentOutput, -speed);
			rr -> Set(ControlMode :: PercentOutput, speed);
		}
	}
	void StrafeStraight(std::string side)
		{
			double slop = 5;
			double setAngle;
			double rotSpeed = 0;
			double thisAuton = 0.6;
			double currentAngle = gyro->GetYaw();

			if(side == "right")
			{
				if(done == false)
				{
					setAngle = currentAngle;
					done = true;
				}

				float angDiff = setAngle - currentAngle;

				if(angDiff < -slop)
					rotSpeed = -thisAuton/3;


				if(angDiff > slop)
					rotSpeed = thisAuton/3;

			}
			if(side == "left")
			{
				if(done == false)
				{
					setAngle = currentAngle;
					done = true;
				}

				float angDiff = setAngle - currentAngle;

				thisAuton = -thisAuton;

				if(angDiff < -slop)
					rotSpeed = thisAuton/3;

				if(angDiff > slop)
					rotSpeed = -thisAuton/3;

			}
			AutonMecDrive(thisAuton, 0, rotSpeed);
		}
	void Strafe(std::string side)
	{
		if(side == "left"){
			lf ->Set(ControlMode :: PercentOutput, autonSpeed);
			lr ->Set(ControlMode :: PercentOutput, -autonSpeed);
			rf ->Set(ControlMode :: PercentOutput, autonSpeed);
			rr ->Set(ControlMode :: PercentOutput, -autonSpeed);
		}
		else if(side == "right"){
			lf ->Set(ControlMode :: PercentOutput, -autonSpeed);
			lr ->Set(ControlMode :: PercentOutput, autonSpeed);
			rf ->Set(ControlMode :: PercentOutput, -autonSpeed);
			rr ->Set(ControlMode :: PercentOutput, autonSpeed);
		}
	}
	void Lifter(int x)
	{
		//raise or lower the lifting device
		if (x == 1)
			sLift->Set(-1);

		else if(x==0)
			sLift->Set(-.2);

	}
	bool turnComplete = false;


	void Turn(int wantAngle)
	{
//		frc::SmartDashboard::PutNumber("Want Angle", wantAngle);
//		frc::SmartDashboard::PutNumber("Current Angle", CurrentAngle);
		int slop = 5;
		if(wantAngle < currentAngle - slop)
			TurnLeft(autonTurnSpeed);

		else if(wantAngle > currentAngle + slop)
			TurnRight(autonTurnSpeed);

		else{
			Stop();
			turnComplete = true;
			gyro->ZeroYaw();
			currentAngle = 0;
			lr->SetSelectedSensorPosition(0, 0, 0);
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
		if (distance < 44 && distance >= 8)
		driveSpeed = driveSpeed * 0.9;

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
	void ActuateBirdPole(int extend) //this function 1. makes the motor go forward to extend the pole
									 //2. rewinds the pole by making the motor negative 3. stops the motor
		{
			if (extend > 0)
				cExtend->Set(1);


			else if (extend < 0)
				cExtend->Set(-1);


			else if (extend == 0)
				cExtend->Set(0.0);

		}


	void DeployBirdPole()
	{
		if(js1->GetRawButton(4) && isBirdOut == false && isYDown == false)
		{
			birdSol->Set(true);
			isBirdOut = true;
			isYDown = true;
		}
		else if(!js1->GetRawButton(4) && isBirdOut == true)
			isYDown = false;

		else if(js1->GetRawButton(4) && isBirdOut == true && isYDown == false)
		{
			birdSol->Set(false);
			isBirdOut = false;
			isYDown = true;
		}
		else if(!js1->GetRawButton(4) && isBirdOut == false)
			isYDown = false;

	}
	void winch()
		{
			if(js1->GetRawButton(1))
			{
				cWinch->Set(-1);
				cWinch2->Set(-1);
			}

			else if(js1->GetRawButton(10))
			{
				cWinch->Set(1);
				cWinch2->Set(1);
			}

			else
			{
				cWinch->Set(0);
				cWinch2->Set(0);
			}
		}

	void FindBlock(){
			if(centerX > -1){
			blockAngle = (180/Pi) * atan((centerX - 200)/FocalLength) + Offset;
			currentAngle = gyro->GetYaw();
			angleTotal = currentAngle + blockAngle;
			SmartDashboard::PutNumber("Total Angle", angleTotal);
			SmartDashboard::PutNumber("Camera Angle", blockAngle);
			SmartDashboard::PutNumber("Current Angle 1", currentAngle);
			SmartDashboard::PutNumber("Block Angle", blockAngle);
			foundBlock = true;
			}
	}

	void teleopSkyLift()
	{
		bool goingUp = js1->GetRawButton(skyLiftUp);
		bool goingDown = js1->GetRawButton(skyLiftDown);

		if(!goingUp && !goingDown)
			sLift ->Set(0);

		else if(goingUp)
		 sLift ->Set(-1);

		else if(goingDown)
			sLift ->Set(1);

	}

	void teleopArmControl() //Secondary controller controls to move grabber
	{
		if(js2->GetRawButton(pickupArmsUp))
			armSol->Set(false);

		if(js2->GetRawButton(pickupArmsDown))
			armSol->Set(true);

		if(js2->GetRawButton(openArms))
			clawSol->Set(true);

		if(js2->GetRawButton(closeArms))
			clawSol->Set(false);

	}

	void Drop()
	{
		armSol->Set(true);
		clawSol->Set(true);
	}

	void autonThrow(double x){
		pWheelL->Set(x);
		pWheelR->Set(-x);
	}

	void teleopGrabToggle()
	{
		if(js2->GetRawButton(4))
		{
			clawSol->Set(true);
		}
		else if(js2->GetRawButton(1))
		{
			clawSol->Set(false);
		}
		else
		{
			clawSol->Set(false);
		}

		if(js2->GetRawButton(5)){
			armSol->Set(true);
		}
		if(js2->GetRawButton(7)){
			armSol->Set(false);
		}
	}

	void pickUpWheels() //Secondary controller controls to turn on wheels
	{
		if(js2->GetRawButton(captureCube)){
			pWheelL->Set(.75);
			pWheelR->Set(-.75);
		}
		else if (js2->GetRawButton(throwCube)){
			pWheelL->Set(-.6);
			pWheelR->Set(.6);
		}
		else{
			pWheelL->Set(0);
			pWheelR->Set(0);
		}
	}

	void LeftOne()
	{
		encoders = true;
	if(encoders){
		currentRot = -1*(lr->GetSelectedSensorPosition(0));
		currentLift = sLift->GetSelectedSensorPosition(0);
		FirstDistance = 85.25;
		LiftHeight = 36;
		firstRot = (int)(((FirstDistance)/ipr)*1.0);
		liftRot = (int)(((LiftHeight)/iprLift)*1.0);

		if(currentRot < firstRot){
			state = 1;
		}

		if(currentRot > firstRot && currentLift < liftRot && state == 1){
			Stop();
			state = 3;
		}

		if(currentLift >= liftRot && !turnComplete && state == 3){
			state = 4;
		}

		if(turnComplete && state == 4){
			state = 5;
		}

	}
	/*
	else if(!encoders){
			FirstAction = (140.0*timeCon);//210
			SecondAction = (72.0*liftCon)+FirstAction;//240

			if(timer < FirstAction){
				state = 1;//Driving from start to First action

			}
			if(timer >= FirstAction && state == 1){
				state = 2;//stopping at first action
			}
			if(timer > FirstAction && timer < SecondAction){
				state = 3;//lifting between first and second action
			}
			if(timer > SecondAction && state == 3 && !turnComplete){
				state = 4;//turning right 90 degrees
			}
			if(turnComplete && state == 4){
				state = 5; //Stopping the motors and drop the cube.
			}
		}
	*/

		if(state == 1){
			DriveStraight(autonSpeed);
		}
		else if(state == 2){
			Stop();
		}
		else if(state == 3){
			Lifter(1);
		}
		else if(state == 4){
			Lifter(0);
			Turn(90);
		}

		else if(state == 5){
			Drop();
			Stop();
		}
	}
	void LeftTwo()
	{
		encoders = false;
		if(encoders){
			currentRot = -1*(lr->GetSelectedSensorPosition(0));
			FirstDistance = 70.0;
			SecondDistance = 155.0;
			ThirdDistance = 70.0;
			firstRot = (int)(((FirstDistance)/ipr)*1.0);
			secondRot = (int)(((SecondDistance)/ipr)*1.0) + firstRot;
			thirdRot = (int)(((ThirdDistance)/ipr)*1.0) + secondRot;

			if(currentRot < firstRot){
				state = 1;
			}
			if(currentRot > firstRot && currentRot < secondRot && state == 1){
				state = 2;
			}
			if(currentRot > secondRot && currentRot < thirdRot && state == 2){
				Stop();
				state = 4;
			}
			if(currentRot > thirdRot && state == 4){
				state = 5;
			}
		}
		else if(!encoders){
			FirstAction = (76.0*timeCon);//originally 30
			SecondAction = (75.0*strafeCon) + FirstAction;//originally 180
			ThirdAction = (141.0*timeCon) + SecondAction;//originally 250

			if(timer < FirstAction){
				state = 1;//Lifting and driving from start to FirstAction

			}
			if(timer > FirstAction && timer < SecondAction){
				state = 2;//Strafing right from First to second action
			}
			if(timer == SecondAction) {
				state = 3;//stopping at second action
			}
			if(timer > SecondAction && timer < ThirdAction){
				state = 4;//Driving from second to third action
			}
			if(timer > ThirdAction){
				state = 5;//Stopping and dropping at third action
			}
		}


		if(state == 1){
			Lifter(1);
			DriveStraight(autonSpeed);
			std::cout<<"lifting and driving"<<std::endl;
		}
		else if(state == 2){
			Lifter(0);
			StrafeStraight("left");
			std::cout<<"strafing"<<std::endl;
		}
		else if(state == 3){
			Stop();
			std::cout<<"stop"<<std::endl;
		}
		else if(state == 4){
			DriveStraight(autonSpeed);
			std::cout<<"driving"<<std::endl;
		}
		else if(state == 5){
			Stop();
			std::cout<<"stop"<<std::endl;
			Drop();
			std::cout<<"drop"<<std::endl;
		}

	}
	void LeftThree()
	{
		encoders = true;
		if(encoders){
			currentRot = -1*(lr->GetSelectedSensorPosition(0));
			currentLift = sLift->GetSelectedSensorPosition(0);
			FirstDistance = 200.0;
			SecondDistance = 172.0;
			OldThirdDistance = 30.0;
			ThirdDistance = 42.0;
			FourthDistance = 14.0;
			firstRot = (int)(((FirstDistance)/ipr)*1.0);
			secondRot = (int)(((SecondDistance)/ipr)*1.0);
			thirdRot = (int)(((ThirdDistance)/ipr)*1.0);
			fourthRot = (int)(((FourthDistance)/ipr)*1.0);
			if(currentRot < firstRot && state == 0){
				state = 1;
			}
			if(currentRot > firstRot && !turnComplete && state == 1){
				Stop();
				state = 3;
			}
			if(turnComplete && state == 3 && currentRot < secondRot){
				currentRot = 0;
				state = 5;
			}

			if( state == 5 && !turnComplete && currentRot >= secondRot){
				currentRot = 0;
				state = 8;
			}
			if(turnComplete && state == 8){
				currentRot = 0;
				state = 10;
			}
			if(currentRot > thirdRot && state == 10){
				state = 11;
			}

			if(clawSol->Get() == true && state == 11)
			{
				state = 12;
			}

			if(currentRot < fourthRot && state == 12)
			{
				state = 13;
			}

		}
		else if(!encoders){
			int FirstAction = (228.375*timeCon);//184
			int SecondAction = (182.75*timeCon)+FirstAction;//284
			int ThirdAction = (72.0*liftCon)+SecondAction;//410
			int FourthAction = (18.805*timeCon)+ThirdAction;//460
			if(timer < FirstAction){
				state = 1;//Drive between start and first action
			}
			if(timer >= FirstAction && state == 1){
				state = 2;//Stopping at first action
			}
			if(timer > FirstAction && state == 2 && !turnComplete){
				state = 3;//Turn left 90 degrees
			}
			if(state == 3 && turnComplete){
				state = 4;//Stop and set timer to first action
			}
			if(timer > FirstAction && timer < SecondAction && state == 4){
				state = 5;//Driving between First and Second action
			}
			if(timer >= SecondAction && state == 5){
				state = 6;//Stopping at second action
			}
			if(timer > SecondAction && timer < ThirdAction && state == 6){
				state = 7;//Lifting from second to third action
			}
			if(state == 7 && timer > ThirdAction && !turnComplete){
				state = 8;//Turning right at third action
			}
			if(state == 8 && turnComplete){
				state = 9; //Stopping and setting timer to third action
			}
			if(timer > ThirdAction && timer < FourthAction && state == 9){
				state = 10;//Driving forward between third and fourth action
				currentRot = 0;
				}
			if(timer > FourthAction && state == 10){
				state = 11;//Stopping and dropping at fourth action
				}
		}

		if (state == 1){
			DriveStraight(autonSpeed);
		}
		else if (state == 2){
			Stop();
		}
		else if (state == 3){
			Turn(-90);
		}
		else if (state == 4){
			Stop();
			timer = FirstAction;
		}
		else if (state == 5){
			turnComplete = false;
			DriveStraight(autonSpeed);
			Lifter(1);
			armSol->Set(true);
		}
		else if (state == 6){
			Stop();
		}
		else if(state == 7){
			Lifter(1);
		}
		else if(state == 8){
			Lifter(0);
			Turn(90);
		}

		else if(state == 9){
			Stop();
			timer = ThirdAction;
			turnComplete = false;
		}
		else if(state == 10){
		DriveStraight(autonSpeed);
		}
		else if(state == 11){
			Stop();
			Drop();
		}

		else if(state == 12)
		{
			DriveStraight(-autonSpeed);
		}

		else if (state == 13)
		{
			Stop();
		}

	}

	void LeftThreeHalf()
		{
			encoders = true;
			if(encoders){
				currentRot = -1*(lr->GetSelectedSensorPosition(0));
				currentLift = sLift->GetSelectedSensorPosition(0);
				FirstDistance = 200.0;
				SecondDistance = 160.0/2;
				ThirdDistance = 30.0;
				firstRot = (int)(((FirstDistance)/ipr)*1.0);
				secondRot = (int)(((SecondDistance)/ipr)*1.0);
				thirdRot = (int)(((ThirdDistance)/ipr)*1.0);
				if(currentRot < firstRot && state == 0){
					state = 1;
				}
				if(currentRot > firstRot && !turnComplete && state == 1){
					Stop();
					state = 3;
				}
				if(turnComplete && state == 3 && currentRot < secondRot){
					state = 5;
				}

				if( state == 5 && currentRot >= secondRot){
					state = 6;
				}

			}
			else if(!encoders){
				int FirstAction = (228.375*timeCon);//184
				int SecondAction = (182.75*timeCon)+FirstAction;//284

				if(timer < FirstAction){
					state = 1;//Drive between start and first action
				}
				if(timer >= FirstAction && state == 1){
					state = 2;//Stopping at first action
				}
				if(timer > FirstAction && state == 2 && !turnComplete){
					state = 3;//Turn left 90 degrees
				}
				if(state == 3 && turnComplete){
					state = 4;//Stop and set timer to first action
				}
				if(timer > FirstAction && timer < SecondAction && state == 4){
					state = 5;//Driving between First and Second action
				}
			}

			if (state == 1){
				DriveStraight(autonSpeed);
			}
			else if (state == 2){
				Stop();
			}
			else if (state == 3){
				Turn(-90);
			}
			else if (state == 4){
				Stop();
				timer = FirstAction;
			}
			else if (state == 5){
				DriveStraight(autonSpeed);
			}
			else if (state == 6)
				Stop();
		}

	void LeftScale()
	{
		encoders = true;
		if(encoders){
			currentRot = -1*(lr->GetSelectedSensorPosition(0));
			currentLift = sLift->GetSelectedSensorPosition(0);
			FirstDistance = 130.0;
			FirstDistanceV2 = 150.0;
			SecondDistance = 9.0;
			ThirdDistance = 6.0;
			LiftHeight = 72;
			liftRot = (int)(((LiftHeight)/iprLift)*1.0);
			firstRot = (int)(((FirstDistance)/ipr)*1.0);
			firstRotV2 = (int)(((FirstDistanceV2)/ipr)*1.0)+firstRot;
			secondRot = (int)(((SecondDistance)/ipr)*1.0);
			thirdRot = (int)(((ThirdDistance)/ipr)*1.0) + secondRot;
			if(currentRot < firstRot && state == 0){
				state = 1;
			}
			if( currentRot > firstRot && state == 1){
				Lifter(1);
				armSol->Set(true);
				//currentLift < liftRot &&
			}
			if(currentRot >= firstRotV2 && !turnComplete && state == 1){
				state = 4;
			}
			if(turnComplete && state == 4){
				state = 5;
			}
			if(currentRot >= secondRot && state == 5){
				state = 6;
				timer = 0;
			}
			if(timer >= 100 && state == 6){
				state = 7;
			}

		}
		else if(!encoders){
			std::cout<<"LeftScale"<<std::endl;
			FirstAction = (290.0*timeCon);//210
			SecondAction = (90.0*liftCon)+FirstAction;//240
			ThirdAction  = (30.0*timeCon)+SecondAction;
			FourthAction = (18.0*timeCon)+ThirdAction;
			std::cout<<state<<std::endl;
			std::cout<<timer<<std::endl;
			std::cout<<FirstAction<<std::endl;
			std::cout<<SecondAction<<std::endl;
			std::cout<<ThirdAction<<std::endl;

			if(timer < FirstAction){
				state = 1;//Driving from start to First action

			}
			if(timer >= FirstAction && state == 1){
				state = 2;//stopping at first action

			}
			if(timer > FirstAction && timer < SecondAction){
				state = 3;//lifting between first and second action

			}
			if(timer > SecondAction && state == 3 && !turnComplete){
				currentRot = 0;
				state = 4;//turning right 90 degrees
			}
			if(turnComplete && state == 4){
				state = 5; //Driving forward a little, between second and third action
				timer = SecondAction;
			}
			if(timer > ThirdAction && timer < FourthAction && state == 5){
				state = 6;

				}
			if(timer >= FourthAction && state == 6){
				state = 7;

			}
		}
		if(state == 1){
			DriveStraight(autonSpeed);
			std::cout<<"driving"<<std::endl;
		}
		else if(state == 2){
			Stop();
			//DriveStraight(autonSpeed);
			std::cout<<"stop"<<std::endl;
		}
		else if(state == 3){
			DriveStraight(autonSpeed);
			Lifter(1);
			armSol->Set(true);
			std::cout<<"lifting"<<std::endl;
		}
		else if(state == 4){
			Lifter(0);
			Turn(90);
			std::cout<<"turning"<<std::endl;
		}
		else if(state == 5){
			DriveStraight(autonSpeed);

			std::cout<<"driving"<<std::endl;
		}
		else if(state == 6){
			autonThrow(0.4);
			DriveStraight(-0.15);
			std::cout<<"ejecting"<<std::endl;
		}
		else if(state == 7){
			Stop();
			autonThrow(0.0);
			std::cout<<"stopping"<<std::endl;
		}
	}
	void LeftScale_DoubleAuton()
			{
				encoders = true;
				if(encoders){
					currentRot = -1*(lr->GetSelectedSensorPosition(0));
					currentLift = sLift->GetSelectedSensorPosition(0);
					FirstDistance = 100.0;
					FirstDistanceV2 = 125.0;
					SecondDistance = 24.0;
					ThirdDistance = 6.0;
					LiftHeight = 72;
					BlockDistance = 54.0;
					liftRot = (int)(((LiftHeight)/iprLift)*1.0);
					firstRot = (int)(((FirstDistance)/ipr)*1.0);
					firstRotV2 = (int)(((FirstDistanceV2)/ipr)*1.0)+firstRot;
					secondRot = (int)(((SecondDistance)/ipr)*1.0);
					thirdRot = (int)(((ThirdDistance)/ipr)*1.0) + secondRot;
					blockRotation = (int)(((BlockDistance)/ipr)*1.0);
					reverseDistance = -36.0;
					reverseRotation = (int)(((reverseDistance)/ipr)*1.0);
					sixthDistance = 36.0;
					sixthRotation = (int)(((sixthDistance)/ipr)*1.0);
					seventhDistance = -12.0;
					SmartDashboard::PutNumber("Block Angle", blockAngle);
					//SmartDashboard::PutNumber("", autonPosition);

					if(currentRot < firstRot && state == 0){
						state = 1;
					}
					if( currentRot > firstRot && state == 1){
						Lifter(1);
						armSol->Set(true);
						//currentLift < liftRot &&
					}
					if(currentRot >= firstRotV2 && !turnComplete && state == 1){
						currentRot = 0;
						state = 4;
					}
					if(turnComplete && state == 4){
						state = 5;
						currentRot = 0;
					}
					if(currentRot >= secondRot && state == 5){
						state = 6;
						timer = 0;
					}
					if(timer >= 50 && state == 6){
						state = 7;
						turnComplete = false;
					}
					if(State7Done && state == 7){
							state = 8; //turn 90 to right
						}
					if(turnComplete && state == 8){
						currentRot = 0;
						state = 9;//finding block angle
					}
					if (foundBlock && state == 9){
						state = 10; //turning toward block angle
						turnComplete = false;
					}
					if(turnComplete && state == 10){
						currentRot = 0;
						state = 11;//drive forward
						timer = 0;
					}
					if((currentRot >= blockRotation || timer>150) && state == 11){
						state = 12;
						timer = 0;
						//stop and grab
					}
					if(timer > 50 && state == 12){
						lr->SetSelectedSensorPosition(0, 0, 0);
						state = 13; //backup
					}
					if(currentRot <= reverseRotation && state == 13){
						state = 14;//turn torward scale
						turnComplete = false;
					}
					if(timer >= 200 && turnComplete && state == 14){
						currentRot = 0;
						state = 15; //drive forward
					}
					if(currentRot >= sixthRotation && state == 15){
						state = 16; //shooting
						timer = 0;
					}
					if(timer >= 100 && state == 16){
						state = 17; //stop
					}
				}
				else if(!encoders){
					std::cout<<"LeftScale"<<std::endl;
					FirstAction = (290.0*timeCon);//210
					SecondAction = (90.0*liftCon)+FirstAction;//240
					ThirdAction  = (30.0*timeCon)+SecondAction;
					FourthAction = (18.0*timeCon)+ThirdAction;
					std::cout<<state<<std::endl;
					std::cout<<timer<<std::endl;
					std::cout<<FirstAction<<std::endl;
					std::cout<<SecondAction<<std::endl;
					std::cout<<ThirdAction<<std::endl;

					if(timer < FirstAction){
						state = 1;//Driving from start to First action

					}
					if(timer >= FirstAction && state == 1){
						state = 2;//stopping at first action

					}
					if(timer > FirstAction && timer < SecondAction){
						state = 3;//lifting between first and second action

					}
					if(timer > SecondAction && state == 3 && !turnComplete){
						state = 4;//turning right 90 degrees

					}
					if(turnComplete && state == 4){
						state = 5; //Driving forward a little, between second and third action
						timer = SecondAction;
					}
					if(timer > ThirdAction && timer < FourthAction && state == 5){
						state = 6;

						}
					if(timer >= FourthAction && state == 6){
						state = 7;

					}
				}
				if(state == 1){
					DriveStraight(autonSpeed);
					std::cout<<"driving"<<std::endl;
				}
				else if(state == 2){
					Stop();
					//DriveStraight(autonSpeed);
					std::cout<<"stop"<<std::endl;
				}
				else if(state == 3){
					DriveStraight(autonSpeed);
					Lifter(1);
					armSol->Set(true);
					std::cout<<"lifting"<<std::endl;
				}
				else if(state == 4){
					Lifter(0);
					Turn(45);
					std::cout<<"turning"<<std::endl;
				}
				else if(state == 5){
					DriveStraight(autonSpeed);

					std::cout<<"driving"<<std::endl;
				}
				else if(state == 6){

					autonThrow(0.7);
					DriveStraight(-0.1);
					std::cout<<"ejecting"<<std::endl;
				}
				else if(state == 7){
					Stop();
					autonThrow(0.0);
					State7Done = true;
					std::cout<<"stopping"<<std::endl;
				}
				else if(state == 8){
					Turn(90);
					std::cout<<"state = 8 turn2"<<std::endl;
				}
				else if(state == 9){
					FindBlock();
					std::cout<<"state=9,findingblockangle"<<std::endl;
				}
				else if(state == 10){
					Turn(angleTotal);
					Lifter(-1);
					clawSol->Set(true);
					SmartDashboard::PutNumber("Angle Total (same)", angleTotal);
					std::cout<<"state=10,turningtoblock"<<std::endl;
				}
				else if(state == 11){
					DriveStraight(0.3);
					Lifter(-1);
					pWheelL->Set(.75);
					pWheelR->Set(-.75);
					std::cout<<"state=11,driving"<<std::endl;
				}
				else if(state == 12){
					Stop();
					clawSol->Set(false);
					std::cout<<"state=12,grabbing block"<<std::endl;
					Lifter(0);
				}
				else if(state == 13){
					DriveStraight(-0.2);
					pWheelL->Set(0);
					pWheelR->Set(0);
					Lifter(1);

					std::cout<<"state=13,reversing"<<std::endl;
				}
				else if(state == 14){
					Turn(-140 + blockAngle);
					Lifter(0);
					std::cout<<"state=14,turning"<<std::endl;
				}
				else if(state == 15){
					DriveStraight(0.2);
					Lifter(0);

					std::cout<<"state=15,driving"<<std::endl;
				}
				else if(state == 16){
					//(0.5);
					Drop();
					DriveStraight(-autonSpeed * 0.5);
					std::cout<<"state=16,ejecting"<<std::endl;
				}
				else if(state == 17){
					Stop();
					std::cout<<"state=17,stopping"<<std::endl;
				}
			}

	void RightOne()
	{
		encoders = true;
		if(encoders){
			currentRot = -1*(lr->GetSelectedSensorPosition(0));
			currentLift = sLift->GetSelectedSensorPosition(0);
			FirstDistance = 200.0;
			SecondDistance = 172.0;
			OldThirdDistance = 30.0;
			ThirdDistance = 42.0;
			FourthDistance = 14.0;
			firstRot = (int)(((FirstDistance)/ipr)*1.0);
			secondRot = (int)(((SecondDistance)/ipr)*1.0);
			thirdRot = (int)(((ThirdDistance)/ipr)*1.0);
			fourthRot = (int)(((FourthDistance)/ipr)*1.0);
			if(currentRot < firstRot && state == 0){
				state = 1;
			}
			if(currentRot > firstRot && !turnComplete && state == 1){
				Stop();
				state = 3;
			}
			if(turnComplete && state == 3 && currentRot < secondRot){
				currentRot = 0;
				state = 5;
			}

			if( state == 5 && !turnComplete && currentRot >= secondRot){
				currentRot = 0;
				state = 8;
			}
			if(turnComplete && state == 8){
				currentRot = 0;
				state = 10;
			}
			if(currentRot > thirdRot && state == 10){
				state = 11;
			}

			if (clawSol->Get() == true && state == 11)
			{
				state = 12;
			}

			if (currentRot < fourthRot && state == 12)
			{
				state = 13;
			}

		}
		else if(!encoders){
			int FirstAction = (228.375*timeCon);//184
			int SecondAction = (182.75*timeCon)+FirstAction;//284
			int ThirdAction = (72.0*liftCon)+SecondAction;//410
			int FourthAction = (18.805*timeCon)+ThirdAction;//460
			if(timer < FirstAction){
				state = 1;//Drive between start and first action
			}
			if(timer >= FirstAction && state == 1){
				state = 2;//Stopping at first action
			}
			if(timer > FirstAction && state == 2 && !turnComplete){
				state = 3;//Turn left 90 degrees
			}
			if(state == 3 && turnComplete){
				state = 4;//Stop and set timer to first action
			}
			if(timer > FirstAction && timer < SecondAction && state == 4){
				state = 5;//Driving between First and Second action
			}
			if(timer >= SecondAction && state == 5){
				state = 6;//Stopping at second action
			}
			if(timer > SecondAction && timer < ThirdAction && state == 6){
				state = 7;//Lifting from second to third action
			}
			if(state == 7 && timer > ThirdAction && !turnComplete){
				state = 8;//Turning right at third action
			}
			if(state == 8 && turnComplete){
				state = 9; //Stopping and setting timer to third action
			}
			if(timer > ThirdAction && timer < FourthAction && state == 9){
				state = 10;//Driving forward between third and fourth action
			}
			if(timer > FourthAction && state == 10){
				state = 11;//Stopping and dropping at fourth action
			}
		}
		if (state == 1){
			DriveStraight(autonSpeed);
			}
		else if (state == 2){
			Stop();
		}
		else if (state == 3){
			Turn(90);
		}
		else if (state == 4){
			Stop();
			timer = FirstAction;
		}
		else if (state == 5){
			turnComplete = false;
			DriveStraight(autonSpeed);
			Lifter(1);
			armSol->Set(true);
		}
		else if (state == 6){
			Stop();
		}
		else if(state == 7){
			Lifter(1);
		}
		else if(state == 8){
			Lifter(0);
			Turn(-90);
		}

			else if(state == 9){
				Stop();
				timer = ThirdAction;
				turnComplete = false;
			}
			else if(state == 10){
				DriveStraight(autonSpeed);
			}
			else if(state == 11){
				Stop();
				Drop();
			}
			else if (state == 12)
			{
				DriveStraight(-autonSpeed);
			}

			else if (state == 13)
			{
				Stop();
			}
	}

	void RightOneHalf()
	{
		encoders = true;
		if(encoders){
			currentRot = -1*(lr->GetSelectedSensorPosition(0));
			currentLift = sLift->GetSelectedSensorPosition(0);
			FirstDistance = 200.0;
			SecondDistance = 160.0/2;
			ThirdDistance = 30.0;
			firstRot = (int)(((FirstDistance)/ipr)*1.0);
			secondRot = (int)(((SecondDistance)/ipr)*1.0);
			thirdRot = (int)(((ThirdDistance)/ipr)*1.0);

			if(currentRot < firstRot && state == 0){
				state = 1;
			}
			if(currentRot > firstRot && !turnComplete && state == 1){
				Stop();
				state = 3;
			}
			if(turnComplete && state == 3 && currentRot < secondRot){
				state = 5;
			}
			if( state == 5 && currentRot >= secondRot){
				state = 6;
			}

		}
		else if(!encoders){
			int FirstAction = (228.375*timeCon);//184
			int SecondAction = (182.75*timeCon)+FirstAction;//284

			if(timer < FirstAction){
				state = 1;//Drive between start and first action
			}
			if(timer >= FirstAction && state == 1){
				state = 2;//Stopping at first action
			}
			if(timer > FirstAction && state == 2 && !turnComplete){
				state = 3;//Turn left 90 degrees
			}
			if(state == 3 && turnComplete){
				state = 4;//Stop and set timer to first action
			}
			if(timer > FirstAction && timer < SecondAction && state == 4){
				state = 5;//Driving between First and Second action
			}

		}
		if (state == 1){
			DriveStraight(autonSpeed);
			}
		else if (state == 2){
			Stop();
		}
		else if (state == 3){
			Turn(90);
		}
		else if (state == 4){
			Stop();
			timer = FirstAction;
		}
		else if (state == 5){
			DriveStraight(autonSpeed);

		} else if (state == 6) {
			Stop();
		}
	}


	void RightTwo()
	{
		encoders = false;
		if(encoders){
			currentRot = -1*(lr->GetSelectedSensorPosition(0));
			FirstDistance = 70.0;
			SecondDistance = 155.0;
			ThirdDistance = 70.0;
			firstRot = (int)(((FirstDistance)/ipr)*1.0);
			secondRot = (int)(((SecondDistance)/ipr)*1.0) + firstRot;
			thirdRot = (int)(((ThirdDistance)/ipr)*1.0) + secondRot;

			if(currentRot < firstRot){
				state = 1;
			}
			if(currentRot > firstRot && currentRot < secondRot && state == 1){
				state = 2;
			}
			if(currentRot > secondRot && currentRot < thirdRot && state == 2){
				Stop();
				state = 4;
			}
			if(currentRot > thirdRot && state == 4){
				state = 5;
			}
		}
		else if(!encoders){
			FirstAction = (76.0*timeCon);//30
			std::cout<<FirstAction<<std::endl;
			SecondAction = (66.0*strafeCon)+FirstAction;//180
			std::cout<<SecondAction<<std::endl;
			ThirdAction = (141.0*timeCon)+SecondAction;//250
			std::cout<<ThirdAction<<std::endl;
			std::cout<<timer<<std::endl;

			if(timer < FirstAction){
				state = 1;//Lifting and driving from start to FirstAction

			}
			if(timer > FirstAction && timer < SecondAction){
				state = 2;//Strafing right from First to second action
			}
			if(timer >= SecondAction && state == 2) {
				state = 3;//stopping at second action
			}
			if(timer > SecondAction && timer < ThirdAction){
				state = 4;//Driving from second to third action
			}
			if(timer > ThirdAction){
				state = 5;//Stopping and dropping at third action
			}
		}



		if(state == 1){
			Lifter(1);
			DriveStraight(autonSpeed);
			std::cout<<"lifting and driving"<<std::endl;
		}
		else if(state == 2){
			Lifter(0);
			StrafeStraight("right");
			std::cout<<"strafing"<<std::endl;
		}
		else if(state == 3){
			Stop();
			std::cout<<"stop"<<std::endl;
		}
		else if(state == 4){
			DriveStraight(autonSpeed);
			std::cout<<"driving"<<std::endl;
		}
		else if(state == 5){
			Stop();
			std::cout<<"stop"<<std::endl;
			Drop();
			std::cout<<"drop"<<std::endl;
		}

	}

	void RightThree()
	{

		if(encoders){
			currentRot = -1*(lr->GetSelectedSensorPosition(0));
			currentLift = sLift->GetSelectedSensorPosition(0);
			FirstDistance = 85.25;
			LiftHeight = 36;
			firstRot = (int)(((FirstDistance)/ipr)*1.0);
			liftRot = (int)(((LiftHeight)/iprLift)*1.0);
			if(currentRot < firstRot){
				state = 1;
			}
			if(currentRot > firstRot && currentLift < liftRot && state == 1){
				Stop();
				state = 3;
			}
			if(currentLift >= liftRot && !turnComplete && state == 3){
				state = 4;
			}
			if(turnComplete && state == 4){
				state = 5;
			}
		}
		else if(!encoders){
			FirstAction = (140.0*timeCon);//210
			SecondAction = (72.0*liftCon)+FirstAction;//240
			if(timer < FirstAction){
				state = 1;//Driving from start to First action
			}
			if(timer >= FirstAction && state == 1){
				state = 2;//stopping at first action
			}
			if(timer > FirstAction && timer < SecondAction){
				state = 3;//lifting between first and second action
			}
			if(timer > SecondAction && state == 3 && !turnComplete){
				state = 4;//turning left 90 degrees
			}
			if(turnComplete && state == 4){
				state = 5; //Stopping the motors and drop the cube.
			}
		}
		if(state == 1){
			DriveStraight(autonSpeed);
		}
		else if(state == 2){
			Stop();
		}
		else if(state == 3){
			Lifter(1);
		}
		else if(state == 4){
			Lifter(0);
			Turn(-90);
		}
		else if(state == 5){
			Stop();
			Drop();
		}
	}
	void RightScale()
	{
		encoders = true;
		if(encoders){
			currentRot = -1*(lr->GetSelectedSensorPosition(0));
			currentLift = sLift->GetSelectedSensorPosition(0);
			FirstDistance = 130.0;
			FirstDistanceV2 = 150.0;
			SecondDistance = 9.0;
			ThirdDistance = 6.0;
			LiftHeight = 72;
			liftRot = (int)(((LiftHeight)/iprLift)*1.0);
			firstRot = (int)(((FirstDistance)/ipr)*1.0);
			firstRotV2 = (int)(((FirstDistanceV2)/ipr)*1.0)+firstRot;
			secondRot = (int)(((SecondDistance)/ipr)*1.0);
			thirdRot = (int)(((ThirdDistance)/ipr)*1.0) + secondRot;
			if(currentRot < firstRot && state == 0){
				state = 1;
			}
			if( currentRot > firstRot && state == 1){
				Lifter(1);
				armSol->Set(true);
				//currentLift < liftRot &&
			}
			if(currentRot >= firstRotV2 && !turnComplete && state == 1){
				state = 4;
			}
			if(turnComplete && state == 4){
				state = 5;
			}
			if(currentRot >= secondRot && state == 5){
				currentRot = 0;
				state = 6;
				timer = 0;
			}
			if(timer >= 100 && state == 6){
				state = 7;
			}
		}
		else if(!encoders){
			std::cout<<"LeftScale"<<std::endl;
			FirstAction = (290.0*timeCon);//210
			SecondAction = (90.0*liftCon)+FirstAction;//240
			ThirdAction  = (30.0*timeCon)+SecondAction;
			FourthAction = (18.0*timeCon)+ThirdAction;
			std::cout<<state<<std::endl;
			std::cout<<timer<<std::endl;
			std::cout<<FirstAction<<std::endl;
			std::cout<<SecondAction<<std::endl;
			std::cout<<ThirdAction<<std::endl;

			if(timer < FirstAction){
				state = 1;//Driving from start to First action

			}
			if(timer >= FirstAction && state == 1){
				state = 2;//stopping at first action

			}
			if(timer > FirstAction && timer < SecondAction){
				state = 3;//lifting between first and second action

			}
			if(timer > SecondAction && state == 3 && !turnComplete){
				state = 4;//turning right 90 degrees

			}
			if(turnComplete && state == 4){
				state = 5; //Driving forward a little, between second and third action
				timer = SecondAction;
			}
			if(timer > ThirdAction && timer < FourthAction && state == 5){
				state = 6;

				}
			if(timer >= FourthAction && state == 6){
				state = 7;

			}
		}
		if(state == 1){
			DriveStraight(autonSpeed);
			std::cout<<"driving"<<std::endl;
		}
		else if(state == 2){
			Stop();
			//DriveStraight(autonSpeed);
			std::cout<<"stop"<<std::endl;
		}
		else if(state == 3){
			DriveStraight(autonSpeed);
			Lifter(1);
			armSol->Set(true);
			std::cout<<"lifting"<<std::endl;
		}
		else if(state == 4){
			Lifter(0);
			Turn(-90);
			std::cout<<"turning"<<std::endl;
		}
		else if(state == 5){
			DriveStraight(autonSpeed);

			std::cout<<"driving"<<std::endl;
		}
		else if(state == 6){
			autonThrow(0.4);
			DriveStraight(-0.15);
			std::cout<<"ejecting"<<std::endl;
		}
		else if(state == 7){
			Stop();
			autonThrow(0.0);
			std::cout<<"stopping"<<std::endl;
		}
	}

	void RightScale_DoubleAuton()
				{
					encoders = true;
					if(encoders){
						currentRot = -1*(lr->GetSelectedSensorPosition(0));
						currentLift = sLift->GetSelectedSensorPosition(0);
						FirstDistance = 100.0;
						FirstDistanceV2 = 125.0;
						SecondDistance = 24.0;
						ThirdDistance = 6.0;
						LiftHeight = 72;
						BlockDistance = 54.0;
						liftRot = (int)(((LiftHeight)/iprLift)*1.0);
						firstRot = (int)(((FirstDistance)/ipr)*1.0);
						firstRotV2 = (int)(((FirstDistanceV2)/ipr)*1.0)+firstRot;
						secondRot = (int)(((SecondDistance)/ipr)*1.0);
						thirdRot = (int)(((ThirdDistance)/ipr)*1.0) + secondRot;
						blockRotation = (int)(((BlockDistance)/ipr)*1.0);
						reverseDistance = -36.0;
						reverseRotation = (int)(((reverseDistance)/ipr)*1.0);
						sixthDistance = 36.0;
						sixthRotation = (int)(((sixthDistance)/ipr)*1.0);
						seventhDistance = -12.0;
						SmartDashboard::PutNumber("Block Angle", blockAngle);
						//SmartDashboard::PutNumber("", autonPosition);

						if(currentRot < firstRot && state == 0){
							state = 1;
						}
						if( currentRot > firstRot && state == 1){
							Lifter(1);
							armSol->Set(true);
							//currentLift < liftRot &&
						}
						if(currentRot >= firstRotV2 && !turnComplete && state == 1){
							currentRot = 0;
							state = 4;
						}
						if(turnComplete && state == 4){
							currentRot = 0;
							state = 5;
						}
						if(currentRot >= secondRot && state == 5){
							state = 6;
							timer = 0;
						}
						if(timer >= 50 && state == 6){
							state = 7;
							turnComplete = false;
						}
						if(State7Done && state == 7){
								state = 8; //turn 90 to right
							}
						if(turnComplete && state == 8){
							currentRot = 0;
							state = 9;//finding block angle
						}
						if (foundBlock && state == 9){
							state = 10; //turning toward block angle
							turnComplete = false;
						}
						if(turnComplete && state == 10){
							currentRot = 0;
							state = 11;//drive forward
							timer = 0;
						}
						if((currentRot >= blockRotation || timer>150) && state == 11){
							state = 12;
							timer = 0;
							//stop and grab
						}
						if(timer > 50 && state == 12){
							lr->SetSelectedSensorPosition(0, 0, 0);
							state = 13; //backup
						}
						if(currentRot <= reverseRotation && state == 13){
							state = 14;//turn torward scale
							turnComplete = false;
						}
						if(timer >= 200 && turnComplete && state == 14){
							currentRot = 0;
							state = 15; //drive forward
						}
						if(currentRot >= sixthRotation && state == 15){
							state = 16; //shooting
							timer = 0;
						}
						if(timer >= 100 && state == 16){
							state = 17; //stop
						}
					}
					else if(!encoders){
						std::cout<<"RightScale"<<std::endl;
						FirstAction = (290.0*timeCon);//210
						SecondAction = (90.0*liftCon)+FirstAction;//240
						ThirdAction  = (30.0*timeCon)+SecondAction;
						FourthAction = (18.0*timeCon)+ThirdAction;
						std::cout<<state<<std::endl;
						std::cout<<timer<<std::endl;
						std::cout<<FirstAction<<std::endl;
						std::cout<<SecondAction<<std::endl;
						std::cout<<ThirdAction<<std::endl;

						if(timer < FirstAction){
							state = 1;//Driving from start to First action

						}
						if(timer >= FirstAction && state == 1){
							state = 2;//stopping at first action

						}
						if(timer > FirstAction && timer < SecondAction){
							state = 3;//lifting between first and second action

						}
						if(timer > SecondAction && state == 3 && !turnComplete){
							state = 4;//turning right 90 degrees

						}
						if(turnComplete && state == 4){
							state = 5; //Driving forward a little, between second and third action
							timer = SecondAction;
						}
						if(timer > ThirdAction && timer < FourthAction && state == 5){
							state = 6;

							}
						if(timer >= FourthAction && state == 6){
							state = 7;

						}
					}
					if(state == 1){
						DriveStraight(autonSpeed);
						std::cout<<"driving"<<std::endl;
					}
					else if(state == 2){
						Stop();
						//DriveStraight(autonSpeed);
						std::cout<<"stop"<<std::endl;
					}
					else if(state == 3){
						DriveStraight(autonSpeed);
						Lifter(1);
						armSol->Set(true);
						std::cout<<"lifting"<<std::endl;
					}
					else if(state == 4){
						Lifter(0);
						Turn(-45);
						std::cout<<"turning"<<std::endl;
					}
					else if(state == 5){
						DriveStraight(autonSpeed);

						std::cout<<"driving"<<std::endl;
					}
					else if(state == 6){
						autonThrow(0.4);
						DriveStraight(-0.1);
						std::cout<<"ejecting"<<std::endl;
					}
					else if(state == 7){
						Stop();
						autonThrow(0.0);
						State7Done = true;
						std::cout<<"stopping"<<std::endl;
					}
					else if(state == 8){
						Turn(-90);
						std::cout<<"state = 8 turn2"<<std::endl;
					}
					else if(state == 9){
						FindBlock();
						std::cout<<"state=9,findingblockangle"<<std::endl;
					}
					else if(state == 10){
						Turn(angleTotal);
						Lifter(-1);
						clawSol->Set(true);
						SmartDashboard::PutNumber("Angle Total (same)", angleTotal);
						std::cout<<"state=10,turningtoblock"<<std::endl;
					}
					else if(state == 11){
						DriveStraight(0.3);
						Lifter(-1);
						pWheelL->Set(.75);
						pWheelR->Set(-.75);
						std::cout<<"state=11,driving"<<std::endl;
					}
					else if(state == 12){
						Stop();
						clawSol->Set(false);
						std::cout<<"state=12,grabbing block"<<std::endl;
						Lifter(0);
					}
					else if(state == 13){
						DriveStraight(-0.2);
						pWheelL->Set(0);
						pWheelR->Set(0);
						Lifter(1);

						std::cout<<"state=13,reversing"<<std::endl;
					}
					else if(state == 14){
						Turn(140 + blockAngle);
						Lifter(0);
						std::cout<<"state=14,turning"<<std::endl;
					}
					else if(state == 15){
						DriveStraight(0.2);
						Lifter(0);

						std::cout<<"state=15,driving"<<std::endl;
					}
					else if(state == 16){
						Drop();
						DriveStraight(-autonSpeed * 0.5);
						std::cout<<"state=16,ejecting"<<std::endl;
					}
					else if(state == 17){
						Stop();
						std::cout<<"state=17,stopping"<<std::endl;
					}
				}

	void AutonLine()
	{
		if(encoders){
			FirstDistance = 100.0;
			currentRot = -1*lr->GetSelectedSensorPosition(0);
			firstRot = (int)(((FirstDistance)/ipr)*1.0);
			if(currentRot < firstRot){
				state = 1;
			}
			if(currentRot > firstRot){
				state = 2;
			}
		}
		else if(!encoders){
			FirstAction = (266.0*timeCon);

			if(timer < FirstAction){
				state = 1;
			}
			if(timer >= FirstAction && state == 1){
				state = 2;
			}
		}

		if(state == 1){
			DriveStraight(autonSpeed); //Driving, between start and first action
		}
		if(state == 2){
			Stop();// Stopping at first action
		}
	}
	void AutonTesting()
	{
		DriveForward(.5, 50);
	}

	void MecDrive(double xAxis, double yAxis, double rot) //homemade mecanum drive!
	{
		double noMove = 0.2; //Dead area of the axes
		double maxSpeed = .5; //normal speed (not turbo)

		if (fabs(xAxis) < noMove)
			xAxis = 0.0;

		if (fabs(yAxis) < noMove)
			yAxis = 0.0;

		if (fabs(rot) < noMove)
			rot = 0.0;

		if (js1->GetRawButton(turboButton))
			maxSpeed = 1;

		else
			maxSpeed = .5;

		double lfSpeed = -yAxis - xAxis - rot;
		double rfSpeed = +yAxis - xAxis - rot;
		double rrSpeed = +yAxis + xAxis - rot;
		double lrSpeed = -yAxis + xAxis - rot;

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
//	void TankDrive(double xAxis, double yAxis, double zAxis)
//	{
//		double noMove = 0.2; //Dead area of the axes
//		double maxSpeed = .5;
//
//		double lfSpeed = 0;
//		double rfSpeed = 0;
//		double lrSpeed = 0;
//		double rrSpeed = 0;
//
//
//		lf -> Set(ControlMode::PercentOutput, lfSpeed*maxSpeed);
//		lr -> Set(ControlMode::PercentOutput, -lrSpeed*maxSpeed);
//		rf -> Set(ControlMode::PercentOutput, rfSpeed*maxSpeed);
//		rr -> Set(ControlMode::PercentOutput, -rrSpeed*maxSpeed);
//
//	}
	void AutonMecDrive(double xAxis, double yAxis, double rot)
		{
			double lfSpeed = -yAxis - xAxis - rot;
			double rfSpeed = +yAxis - xAxis - rot;
			double rrSpeed = +yAxis + xAxis - rot;
			double lrSpeed = -yAxis + xAxis - rot;

			if (fabs(lfSpeed) > 1)
				lfSpeed = fabs(lfSpeed) / lfSpeed;

			if (fabs(lrSpeed) > 1)
				lrSpeed = fabs(lrSpeed) / lrSpeed;

			if (fabs(rfSpeed) > 1)
				rfSpeed = fabs(rfSpeed) / rfSpeed;

			if (fabs(rrSpeed) > 1)
				rrSpeed = fabs(rrSpeed) / rrSpeed;

			lf -> Set(ControlMode::PercentOutput, lfSpeed);
			lr -> Set(ControlMode::PercentOutput, lrSpeed);
			rf -> Set(ControlMode::PercentOutput, rfSpeed);
			rr -> Set(ControlMode::PercentOutput, rrSpeed);
		}


	void TestAll()
	{
		lf -> Set(ControlMode::PercentOutput, 0.5);
		Wait(twoSeconds);
		lf -> Set(ControlMode::PercentOutput, 0);

		rf -> Set(ControlMode::PercentOutput, 0.5);
		Wait(twoSeconds);
		rf -> Set(ControlMode::PercentOutput, 0);

		lr -> Set(ControlMode::PercentOutput, 0.5);
		Wait(twoSeconds);
		lr -> Set(ControlMode::PercentOutput, 0);

		rr -> Set(ControlMode::PercentOutput, 0.5);
		Wait(twoSeconds);
		rr -> Set(ControlMode::PercentOutput, 0);

		cWinch -> Set(ControlMode::PercentOutput, .5);
		cWinch2 -> Set(ControlMode::PercentOutput, -.5);
		Wait(twoSeconds);
		cWinch -> Set(ControlMode::PercentOutput, -.5);
		cWinch2 -> Set(ControlMode::PercentOutput, .5);
		Wait(twoSeconds);
		cWinch -> Set(ControlMode::PercentOutput, 0);
		cWinch2 ->Set(ControlMode::PercentOutput, 0);
		sLift -> Set(ControlMode::PercentOutput, -1);
		Wait(twoSeconds);
		sLift -> Set(ControlMode::PercentOutput, .5);
		Wait(twoSeconds);
		sLift -> Set(ControlMode::PercentOutput, 0);

		cExtend -> Set(ControlMode::PercentOutput, .5);
		Wait(twoSeconds);
		cExtend -> Set(ControlMode::PercentOutput, -.5);
		Wait(twoSeconds);
		cExtend -> Set(ControlMode::PercentOutput, 0);

		pWheelL -> Set(ControlMode::PercentOutput, .5);
		Wait(twoSeconds);
		pWheelL -> Set(ControlMode::PercentOutput, 0);
		pWheelR -> Set(ControlMode::PercentOutput, .5);
		Wait(twoSeconds);
		pWheelR -> Set(ControlMode::PercentOutput, 0);

		armSol->Set(true);
		Wait(oneSecond);
		armSol->Set(false);
		Wait(oneSecond);

		clawSol->Set(true);
		Wait(oneSecond);
		clawSol->Set(false);
		Wait(oneSecond);

		birdSol->Set(true);
		Wait(oneSecond);
		birdSol->Set(false);
	}

	void ArcadeDrive(double yAxis, double rot)
	{
		double noMove = 0.2; //Dead area of the axes
		double maxSpeed = .5; //normal speed (not turbo)

		if (fabs(rot) < noMove)
			rot = 0.0;

		if (js1->GetRawButton(turboButton))
			maxSpeed = 1;

		else
			maxSpeed = 0.5;

		double lfSpeed = -yAxis - rot;
		double rfSpeed = +yAxis - rot;
		double rrSpeed = +yAxis - rot;
		double lrSpeed = -yAxis - rot;

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

    double Dz(double axisVal) //deadzone value
    {
    	if(axisVal < -0.20)
    		return axisVal;
    	if(axisVal > 0.20)
    		return axisVal;
    	return 0;
    }



	char givenPos = ' '; //Where we start against the alliance wall
	char switchRL = ' '; //Where our side if the switch is

	void AutonomousInit() override
	{
		autonThrow(0);
		Lifter(0);
		state = 0;
		lr->SetSelectedSensorPosition(0, 0, 0);
		rr->SetSelectedSensorPosition(0, 0, 0);
		sLift->SetSelectedSensorPosition(0, 0, 0);
		armSol->Set(false);
		clawSol->Set(false);
		done = false;
		LiftComplete = false;
		timer = 0;
		robotPos = posChooser.GetSelected();
		encoders = encodeChooser.GetSelected();
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		//m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
//		givenPos = robotPos+switchRL;
		//std::cout << "Auto selected: " << m_autoSelected << std::endl;
		gyro->ZeroYaw();
		turnComplete = false;/*
		currentAngle = 0;
		//SmartDashboard::PutNumber("Auton Position", autonPosition);
		//SmartDashboard::PutBoolean("Auton Is Blue Alliance?", autonIsBlueAlliance);
		//SmartDashboard::PutBoolean("Auton Is Going to Left Switch?", autonIsLeftSwitch);

		//SmartDashboard::GetNumber("Auton Position", autonPosition);
		//SmartDashboard::GetBoolean("Auton Is Blue Alliance", autonIsBlueAlliance);

		//if (autonIsBlueAlliance == true)
		//{
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
//			if ()/*left switch is blue*/
//			{
//				autonIsLeftSwitch = true;
//			}
		//}

		//else if(autonIsBlueAlliance == false)
		//{
//			if(/*left switch is red*/)
//			{
//				autonIsLeftSwitch = true;
//			}
		//}

		//SmartDashboard::GetBoolean("Auton Is Going to Left Switch?", autonIsLeftSwitch);

		currentAngle = 0;
		gyro->ZeroYaw();
		}

	void AutonomousPeriodic()
	{
		LiftsensorPos = sLift->GetSelectedSensorPosition(0);
		std::cout<<LiftsensorPos<<std::endl;
		LRsensorPos = lr->GetSelectedSensorPosition(0);
		LFsensorPos = lf->GetSelectedSensorPosition(0);
		RRsensorPos = rr->GetSelectedSensorPosition(0);
		RFsensorPos = rf->GetSelectedSensorPosition(0);
		LiftsensorPos = sLift->GetSelectedSensorPosition(0);
		SmartDashboard::PutNumber("LR Encoder Value", LRsensorPos);
		std::cout<<LRsensorPos<<std::endl;

		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

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
		//std::cout<<currentAngle<<std::endl;
		//SmartDashboard::GetNumber("Auton Position", autonPosition);
		//SmartDashboard::GetBoolean("Auton Is Blue Alliance", autonIsBlueAlliance);
		//SmartDashboard::GetBoolean("Auton Is Going to Left Switch?", autonIsLeftSwitch);

		//autonSpeed = SmartDashboard::GetNumber("autonSpeed", autonSpeed);
		FirstAction = SmartDashboard::GetNumber("First Action Space", FirstAction);
		SecondAction = SmartDashboard::GetNumber("Second Action Space", SecondAction);
		ThirdAction = SmartDashboard::GetNumber("Third Action Space", ThirdAction);
		FourthAction = SmartDashboard::GetNumber("Fourth Action Space", FourthAction);
		FifthAction = SmartDashboard::GetNumber("Fifth Action Space", FifthAction);
		SixthAction = SmartDashboard::GetNumber("Sixth Action Space", SixthAction);

		frc::SmartDashboard::PutNumber("distance", distance);

		timer ++;
		LeftDifference = currentAngle*.05;
		RightDifference = currentAngle*.05;
		currentAngle = gyro->GetYaw(); //Getting what angle we are at
		SmartDashboard::PutNumber(  "CurrentAngle", currentAngle);
		frc::SmartDashboard::PutNumber("Timer", timer);
		frc::SmartDashboard::PutBoolean("TurnComplete?", turnComplete);
		frc::SmartDashboard::PutNumber("State", state);
		frc::SmartDashboard::PutNumber("Right Diff", RightDifference);
		frc::SmartDashboard::PutString("gameData", gameData);
		std::cout<<state<<std::endl;

		halfFarScale = halfwayChooser.GetSelected();
		DoubleScale = doubleChooser.GetSelected();

		pWheelL -> Set(ControlMode::PercentOutput, -.15);
		pWheelR -> Set(ControlMode::PercentOutput, .15);

			if(gameData.length() > 0)
			{
				if(robotPos == 0)
				{
					AutonLine();
					std::cout<<"auto line"<<std::endl;
				}

				else if(gameData[1] == 'L' && robotPos == 1)// && DoubleScale == false)
				{
					LeftScale();
					std::cout<<"Left Scale"<<std::endl;
				}
		/*
				else if(gameData[1] == 'L' && robotPos == 1 && DoubleScale == true)
				{
					LeftScale_DoubleAuton();
					std::cout << "Double Left Scale" << std::endl;
				}
		*/
				else if(gameData[1] == 'R' && robotPos == 3)// && DoubleScale == false)
				{
					RightScale();
					std::cout<<"Right Scale"<<std::endl;
				}

		/*		else if(gameData[1] == 'R' && robotPos == 3 && DoubleScale == true)
				{
					RightScale_DoubleAuton();
					std::cout << "Right Double Scale" << std::endl;
				}
		*/
				else if(gameData[1] == 'L' && robotPos == 3)
				{
					if(halfFarScale == true)
						LeftThreeHalf();

					else
						LeftThree();

					std::cout<<"Left Three - Robot on right, scale on left"<<std::endl;

				}
				else if(gameData[1] == 'R' && robotPos == 1)
				{
					if(halfFarScale == true)
						RightOneHalf();

					else
						RightOne();

					std::cout<<"Right One - Robot on Left, scale on Right"<<std::endl;

				}
				else if(gameData[0] == 'L' && robotPos == 2){
					LeftTwo();
					std::cout<<"Left Two - Robot in center, switch on left"<<std::endl;
				}
				else if(gameData[0] == 'R' && robotPos == 2){
					RightTwo();
					std::cout << "Right Two - Robot in center, switch on left"<<std::endl;
				}
				else{
					AutonLine();
				}



			}
		}




	void TeleopInit()
	{
		gyro->Reset();
		isBirdOut = false;
		isYDown = false;
	}

	void TeleopPeriodic()
	{

		//teleDrive();

//		double jsY=js1->GetRawAxis(joystickY);
//		double jsRot=js1->GetRawAxis(joystickRot);
//		ArcadeDrive(-jsY,jsRot);

		double jsY=js1->GetRawAxis(joystickY);
		double jsX=js1->GetRawAxis(joystickX);
		double jsRot=js1->GetRawAxis(joystickRot);
		MecDrive(jsX,-jsY,jsRot);

		//teleopGrabToggle();

		teleopArmControl();
		//Drop();
		pickUpWheels();
		teleopSkyLift();
		DeployBirdPole();
		winch();

		int PoleExtend=0;
		if (js1->GetRawButton(extendBird)){ //Button B extends the pole
			PoleExtend=1;
		}
		if (js1->GetRawButton(retractBird)){ //Button A retracts the pole
			PoleExtend=-1;
		}

		ActuateBirdPole(PoleExtend);
	}



	void TestPeriodic()
	{
		//if(js1->GetRawButton(1){TestAll();})
		//testWinch();
//		LRsensorPos = lr->GetSelectedSensorPosition(0);
//		LFsensorPos = lf->GetSelectedSensorPosition(0);
//		RRsensorPos = rr->GetSelectedSensorPosition(0);
//		RFsensorPos = rf->GetSelectedSensorPosition(0);
//		SmartDashboard::PutNumber("LR Encoder Value", LRsensorPos);
//		SmartDashboard::PutNumber("RR Encoder Value", RRsensorPos);
//		SmartDashboard::PutNumber("LF Encoder Value", LFsensorPos);
//		SmartDashboard::PutNumber("RF Encoder Value", RFsensorPos);
//		std::cout<<LRsensorPos<<std::endl;
//		std::cout<<LFsensorPos<<std::endl;
//		std::cout<<RRsensorPos<<std::endl;
//		std::cout<<RFsensorPos<<std::endl;

	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	//const std::string kAutoNameDefault = "Default";
	//const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)


