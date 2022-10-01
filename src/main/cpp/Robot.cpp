// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"
#include "Drivetrain.hpp"
#include "units/angular_velocity.h"
#include "units/velocity.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot(){}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::RobotInit() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  Drivetrain::init();
}

void Robot::TeleopPeriodic()
{
  SwerveDrive(false);
  }

void Robot::TestInit() {
  Drivetrain::init();
}

void Robot::TestPeriodic()
{
  // Now introducing Whack-A-Mole.cpp!
  // A new and revolutionary way to lose your sanity
  // Now with null more SIGSEVs!

  const units::meters_per_second_t in_x{1.0};
  const units::meters_per_second_t in_y{1.0};
  const units::degree_t theta{0.5};
  // Yes, officer. This line, right here.
  Drivetrain::faceDirection(in_x, in_y, theta, false);
}


void Robot::SwerveDrive(bool const &field_relative)
{

  units::meters_per_second_t const left_right = -(m_stick.GetLeftX()) * Drivetrain::TELEOP_MAX_SPEED;
  units::meters_per_second_t const front_back = -(m_stick.GetLeftY()) * Drivetrain::TELEOP_MAX_SPEED;
  if (m_stick.GetLeftTriggerAxis() >= 0.0)
    {
    Drivetrain::faceDirection(front_back, left_right, 0_deg, field_relative);
    }

  else if (m_stick.GetRightTriggerAxis() >= 0.0)
    {
    Drivetrain::faceDirection(front_back, left_right, 180_deg, field_relative);
    }
  else if (m_stick.GetAButtonPressed())
    {
    Drivetrain::faceClosest(front_back, left_right, field_relative);
    }
  else if (m_stick.GetRightY())
  {
    // Multiplied by 10 to avoid rounding to 0 by the atan2() method
    double const rotate_joy_x = m_stick.GetRightX() * 10;
    double const rotate_joy_y = -m_stick.GetRightY() * 10;

    // If we aren't actually pressing the joystick, leave rotation at previous
    if (std::abs(rotate_joy_x) > 0.1 || std::abs(rotate_joy_y) > 0.1)
    {
      // Get degree using arctan, then convert from unit circle to front-centered values with positive being CW
      Drivetrain::faceDirection(front_back, left_right, -units::radian_t{atan2(rotate_joy_y, rotate_joy_x)} + 90_deg, field_relative);
    }
    else
      Drivetrain::drive(front_back, left_right, units::radians_per_second_t{0}, field_relative);
  }
  else
  {
    auto const rot = -m_stick.GetLeftX() * Drivetrain::TELEOP_MAX_ANGULAR_SPEED;

    Drivetrain::drive(front_back, left_right, rot, field_relative);
  }
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
