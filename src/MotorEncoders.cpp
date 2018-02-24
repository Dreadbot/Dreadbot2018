/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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

class Robot : public frc::IterativeRobot {
public:

	WPI_TalonSRX *lf; /*left front */
	WPI_TalonSRX *rf; /*right front */
	WPI_TalonSRX *lr;/*left rear */
	WPI_TalonSRX *rr; /*right rear */

	int loopidx = 0;
	int toums = 10;
	double P = 0.1;
	double I = 0.0;
	double D = 0.0;
	double peakPOWER = .4;

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


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
	void AutonomousInit() override {
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {


		int rotationNumb = (2*4096);
				lf->Set(ControlMode::Position,rotationNumb);
				rf->Set(ControlMode::Position,rotationNumb);
				lr->Set(ControlMode::Position,rotationNumb);
				rr->Set(ControlMode::Position,rotationNumb);
	}

	void TeleopPeriodic() {}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
