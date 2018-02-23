/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//I love you old code!
#include <iostream>
#include <string>
#include <SmartDashboard/SmartDashboard.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"
int CurrentAngle;
double autonSpeed = 0.25;
int FirstAction = 100;
int SecondAction = 200;
int ThirdAction = 350;//change back to 400 later
int FourthAction = 550;
int FifthAction = 600;
int SixthAction = 625;
double LeftDifference = 0;
double RightDifference = 0;
//#include "C:\Users\3656-1\wpilib\user\cpp\include\ctre\phoenix\MotorControl\CAN\TalonSRX.h"
using namespace std;
class Robot : public frc::IterativeRobot {

	Ultrasonic *ultrarf = new Ultrasonic (0, 1);
	double distancerf = 50;
	Ultrasonic *ultralf = new Ultrasonic (2, 3);
	double distancelf = 50;
	Ultrasonic *ultrars = new Ultrasonic (4, 5);
	double distancers = 50;
	Ultrasonic *ultrals = new Ultrasonic (6, 7);
	double distancels = 50;

	int timer;
	int state;
	WPI_TalonSRX *lf = new WPI_TalonSRX(0);
	WPI_TalonSRX *rf = new WPI_TalonSRX(1);
	WPI_TalonSRX *lr = new WPI_TalonSRX(2);
	WPI_TalonSRX *rr = new WPI_TalonSRX(3);

	AHRS *gyro = new AHRS(SPI::Port::kMXP);

public:


	 Compressor *c = new Compressor(0);
	 	   Joystick *Controller1 = new Joystick(0);
	       DoubleSolenoid *DSol1 = new DoubleSolenoid(0,1);

	       DoubleSolenoid *DSol2 = new DoubleSolenoid(2,3);

	void RobotInit() {
		c->SetClosedLoopControl(true);
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

			ultrarf->SetAutomaticMode(true);
			ultralf->SetAutomaticMode(true);
			ultrars->SetAutomaticMode(true);
			ultrals->SetAutomaticMode(true);

		gyro->ZeroYaw();
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
	void displayultra()
	{
		distancerf=ultrarf->GetRangeInches();
		distancerf=ultralf->GetRangeInches();
		distancerf=ultrars->GetRangeInches();
		distancerf=ultrals->GetRangeInches();
		frc::SmartDashboard::PutNumber("distancerf",distancerf);
		frc::SmartDashboard::PutNumber("distancelf",distancelf);
		frc::SmartDashboard::PutNumber("distancers",distancers);
		frc::SmartDashboard::PutNumber("distancels",distancels);
	}

	void wait(int t)
	{

	}
	void DriveForward(double speed)
	{
		lf -> Set(ControlMode :: PercentOutput, -speed);
		rf -> Set(ControlMode :: PercentOutput, speed);
		lr -> Set(ControlMode :: PercentOutput, -speed);
		rr -> Set(ControlMode :: PercentOutput, speed);
	}
	void DriveStraight(double speed){
		float slop = 5;

	if(CurrentAngle < 0 - slop){
		lf -> Set(ControlMode :: PercentOutput, -speed);
		rf -> Set(ControlMode :: PercentOutput, speed+LeftDifference);
		lr -> Set(ControlMode :: PercentOutput, -speed);
		rr -> Set(ControlMode :: PercentOutput, speed+LeftDifference);
	}
	else if(CurrentAngle > 0 + slop){
		lf -> Set(ControlMode :: PercentOutput, -speed+RightDifference);
		rf -> Set(ControlMode :: PercentOutput, speed);
		lr -> Set(ControlMode :: PercentOutput, -speed+RightDifference);
		rr -> Set(ControlMode :: PercentOutput, speed);

	}

	else{
		lf -> Set(ControlMode :: PercentOutput, -speed);
		rf -> Set(ControlMode :: PercentOutput, speed);
		lr -> Set(ControlMode :: PercentOutput, -speed);
		rr -> Set(ControlMode :: PercentOutput, speed);
		}
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
	bool turnComplete = false;
	void Turn(int wantAngle)
	{
//		frc::SmartDashboard::PutNumber("Want Angle", wantAngle);
//		frc::SmartDashboard::PutNumber("Current Angle", CurrentAngle);
		int slop = 5;
		if(wantAngle < CurrentAngle - slop){
			TurnLeft(autonSpeed);
		}
		else if(wantAngle > CurrentAngle + slop){
			TurnRight(autonSpeed);
		}
		else{
			Stop();
			turnComplete = true;
			gyro->ZeroYaw();
			CurrentAngle = 0;
		}
	}

	void Lifter(int height){
		if (height == 1)
		{
			//lift to the high place real good-like
		}
		else if(height == 0)
		{
			//lift to the low place real good like
		}
		else
		{
			//pick up cube real awesome like
		}
	}
	void Drop(){
		// Drops the cube
	}
	void DriveSideways(int x, int t)
	{
		//Turn on motors
		wait(t);
		//turn off motors
	}
	void LeftOne()
	{
		if(timer < FirstAction){
			state = 1;

		}
		if(timer == FirstAction){
			state = 2;
		}
		if(timer > FirstAction && state == 2){
			state = 3;
		}
		if(state == 3 && turnComplete){
			state = 4;
		}
		if(state == 4 && timer < SecondAction){
			state = 5;
		}
		if(state == 5 && timer > SecondAction && timer < ThirdAction){
			state = 6;
		}
		if(state == 6 && timer == ThirdAction){
			state = 7;
		}
		if(state == 7 && timer > ThirdAction){
			state = 8;
		}
		if(state == 8 && turnComplete){
			state = 9;
		}
		if(state == 9 && timer >= FourthAction && timer < FifthAction){
			state = 10;
		}
		if(state == 10 && timer == FifthAction){
			state = 11;
		}
		if(state == 11 && timer > FifthAction && timer < SixthAction){
			state = 12;
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
			Lifter(0);
		}
		else if (state == 6){
			DriveStraight(autonSpeed);
		}
		else if(state == 7){
			Stop();
		}
		else if(state == 8){
			Turn(-90);
		}

		else if(state == 9){
			Stop();
			timer = FourthAction;
			turnComplete = false;
		}
		else if(state == 10){
			DriveStraight(autonSpeed);
		}
		else if (state == 11){
			Stop();
		}
		else if (state == 12){
			Drop();
		}
	}

	void LeftTwo()
	{
		if(timer < FirstAction){
			state = 1;
			timer = FirstAction;
		}
		if(timer > FirstAction && timer < SecondAction && state == 1){
			state = 2;
		}
		if(timer > SecondAction && timer < ThirdAction && state == 2) {
			state = 3;
		}
		if(timer == ThirdAction){
			state = 4;
		}
		if(timer > ThirdAction && timer < FourthAction && state == 3){
			state = 5;
		}
		if(timer > FourthAction && timer < FifthAction && state == 4){
			state = 6;
		}
		if(timer == FifthAction) {
			state = 7;
		}
		if(timer < SixthAction && state == 7){
			state = 8;
		}
		if(timer == SixthAction){
			state = 9;
		}

		if(state == 1){
			Lifter(1);
		}
		else if(state == 2){
			Turn(-90);
		}
		else if(state == 3){
			DriveStraight(autonSpeed);
		}
		else if(state == 4){
			Stop();
		}
		else if(state == 5){
			Turn(90);
		}
		else if(state == 6){
			DriveStraight(autonSpeed);
		}
		else if(state == 7){
			Stop();
		}
		else if(state == 8){
			Turn(90);
		}
		else if(state == 9){
			Drop();
		}
		Lifter(1);
		Turn(-90);
		DriveStraight(autonSpeed);
		Turn(90);
		DriveStraight(autonSpeed);
	}

	void LeftThree()
	{
		FirstAction = 50;
		SecondAction = 100;
		ThirdAction = 150;
		FourthAction = 200;

		if(timer < FirstAction){
			state = 1;
			timer = FirstAction;
		}
		if(timer < SecondAction && state == 1 && timer > FirstAction){
			state = 2;
		}
		if(timer == SecondAction && state == 2){
			state = 3;
		}
		if(state == 3){
			state = 4;
		}
		if(state == 4 && turnComplete){
			state = 5;
		}

		if(state == 1){
			Lifter(1);
		}
		else if(state == 2){
			DriveStraight(autonSpeed);
		}
		else if(state == 3){
			Stop();
		}
		else if(state == 4){
			Turn(90);
		}
		else if(state == 5){
			Drop();
		}

	}
	void RightOne()
	{
		FirstAction = 50;
		SecondAction = 100;
		ThirdAction = 150;
		FourthAction = 200;

		if(timer < FirstAction){
			state = 1;
			timer = FirstAction;
		}
		if(timer < SecondAction && state == 1 && timer > FirstAction){
			state = 2;
		}
		if(timer == SecondAction && state == 2){
			state = 3;
		}
		if(state == 3){
			state = 4;
		}
		if(state == 4 && turnComplete){
			state = 5;
		}

		if(state == 1){
			Lifter(1);
		}
		else if(state == 2){
			DriveStraight(autonSpeed);
		}
		else if(state == 3){
			Stop();
		}
		else if(state == 4){
			Turn(-90);
		}
		else if(state == 5){
			Drop();
		}

	}
//
//	void RightTwo()
//	{
//		Lifter(1);
//		wait(5);
//		DriveSideways(-1, 1);
//		DriveForward(1);;
//		Drop();
//	}
//
	void RightThree()
	{
		if(timer < FirstAction){
					state = 1;

		}
		if(timer == FirstAction){
			state = 2;
		}
		if(timer > FirstAction && state == 2){
			state = 3;
		}
		if(state == 3 && turnComplete){
			state = 4;
		}
		if(state == 4 && timer < SecondAction){
			state = 5;
		}
		if(state == 5 && timer > SecondAction && timer < ThirdAction){
			state = 6;
		}
		if(state == 6 && timer == ThirdAction){
			state = 7;
		}
		if(state == 7 && timer > ThirdAction){
			state = 8;
		}
		if(state == 8 && turnComplete){
			state = 9;
		}
		if(state == 9 && timer >= FourthAction && timer < FifthAction){
			state = 10;
		}
		if(state == 10 && timer == FifthAction){
			state = 11;
		}
		if(state == 11 && timer > FifthAction && timer < SixthAction){
			state = 12;
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
			Lifter(0);
		}
		else if (state == 6){
			DriveStraight(autonSpeed);
		}
		else if(state == 7){
			Stop();
		}
		else if(state == 8){
			Turn(90);
		}

		else if(state == 9){
			Stop();
			timer = FourthAction;
			turnComplete = false;
		}
		else if(state == 10){
			DriveStraight(autonSpeed);
		}
		else if (state == 11){
			Stop();
		}
		else if (state == 12){
			Drop();
		}
	}



	int robotPos = 0; // Where robot starts
	int switchRL = 0; // Where the switch is (0 is left, 10 is right)
	int givenPos = 11;
	void AutonomousInit() override {
		timer = 0;
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
//		givenPos = robotPos+switchRL;
		std::cout << "Auto selected: " << m_autoSelected << std::endl;
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		}
		else {
			// Default Auto goes here
		}

		gyro->ZeroYaw();
		turnComplete = false;
		state = 1;
		CurrentAngle = 0;
//		SmartDashboard::PutNumber("autonSpeed", autonSpeed);
//		SmartDashboard::PutNumber("First Action Space", FirstAction);
//		SmartDashboard::PutNumber("Second Action Space", SecondAction);
//		SmartDashboard::PutNumber("Third Action Space", ThirdAction);
//		SmartDashboard::PutNumber("Fourth Action Space", FourthAction);
//		SmartDashboard::PutNumber("Fifth Action Space", FifthAction);
//		SmartDashboard::PutNumber("Sixth Action Space", SixthAction);

	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
		displayultra();
		/*
		distancerf=ultrarf->GetRangeInches();
		distancerf=ultralf->GetRangeInches();
		distancerf=ultrars->GetRangeInches();
		distancerf=ultrals->GetRangeInches();
		frc::SmartDashboard::PutNumber("distancerf",distancerf);
		frc::SmartDashboard::PutNumber("distancelf",distancelf);
		frc::SmartDashboard::PutNumber("distancers",distancers);
		frc::SmartDashboard::PutNumber("distancels",distancels);
*/

		autonSpeed = SmartDashboard::GetNumber("autonSpeed", autonSpeed);
		FirstAction = SmartDashboard::GetNumber("First Action Space", FirstAction);
		SecondAction = SmartDashboard::GetNumber("Second Action Space", SecondAction);
		ThirdAction = SmartDashboard::GetNumber("Third Action Space", ThirdAction);
		FourthAction = SmartDashboard::GetNumber("Fourth Action Space", FourthAction);
		FifthAction = SmartDashboard::GetNumber("Fifth Action Space", FifthAction);
		SixthAction = SmartDashboard::GetNumber("Sixth Action Space", SixthAction);

		timer ++;
		LeftDifference = CurrentAngle*.0125;
		RightDifference = CurrentAngle*.0125;
		CurrentAngle = gyro->GetYaw(); //Getting what angle we are at
		SmartDashboard::PutNumber(  "CurrentAngle", CurrentAngle);
		frc::SmartDashboard::PutNumber("Timer", timer);
		frc::SmartDashboard::PutBoolean("TurnComplete?", turnComplete);
		frc::SmartDashboard::PutNumber("State", state);
		frc:SmartDashboard::PutNumber("Left Diff", LeftDifference);
		frc::SmartDashboard::PutNumber("Right Diff", RightDifference);


//		if(distancerf < 12){
//			lf->Set(ControlMode :: PercentOutput, 0);
//			rf->Set(ControlMode :: PercentOutput, 0);
//			lr->Set(ControlMode :: PercentOutput, 0);
//			rr->Set(ControlMode :: PercentOutput, 0);
//		}
//		else{
//			lf->Set(ControlMode :: PercentOutput, 0.2);
//			rf->Set(ControlMode :: PercentOutput, 0.2);
//			lr->Set(ControlMode :: PercentOutput, -0.2);
//			rr->Set(ControlMode :: PercentOutput, -0.2);
//		}
		switch (givenPos)
		{
		//case 1: LeftOne(); break;
		//case 2: LeftTwo(); break;
		//case 3: LeftThree(); break;
		//case 11: RightOne(); break;
		//case 12: RightTwo(); break;
		//case 13: RightThree(); break;
		default: break;
		}





	}

	void Drive(int xAxis, int yAxis, int rot)
	{

	}


	void TeleopInit() {

	}

	void TeleopPeriodic() {
		frc::SmartDashboard::PutBoolean("Boogie Feild",Controller1->GetRawButton(1));

		displayultra();

		if (Controller1->GetRawButton(1))
					{
						DSol2->Set(DoubleSolenoid::Value::kForward);
						DSol1->Set(DoubleSolenoid::Value::kForward);
					}
				else
				{
					DSol2->Set(DoubleSolenoid::Value::kReverse);
					DSol1->Set(DoubleSolenoid::Value::kReverse);
				}
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
