#pragma once

#include "Drivetrain.hpp"
#include "SwerveModule.hpp"

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

class Robot : public frc::TimedRobot
{
public:
    frc::XboxController m_stick{0};
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    Robot();

    void RobotInit() override;
    void RobotPeriodic() override;
    
    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void SwerveDrive(bool const &field_relative);

private:
};
