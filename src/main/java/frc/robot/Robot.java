// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private final Drivetrain m_drive;
  private final Algae m_algae;
  private final XboxController dj = new XboxController(Constants.oi.kDriverId);
  private final XboxController oj = new XboxController(Constants.oi.kOperId);

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_drive = new Drivetrain();
    m_algae = new Algae();
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_drive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);

    //A button intake algae
    if(dj.getAButtonPressed()) {
      m_algae.intake();
    } else if(dj.getAButtonReleased()) {
      m_algae.stop();
    }

    //B button outtake algae
    if(dj.getBButtonPressed()) {
      m_algae.outtake();
    } else if(dj.getBButtonReleased()) {
      m_algae.stop();
    }

    if(dj.getRightTriggerAxis()>=0.5) {
      m_algae.flipperforward();
    } else if(dj.getRightTriggerAxis()<0.5) {
      m_algae.flipperbackward();
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  private void driveWithJoystick(boolean fieldRelative) {
    final var xSpeed = -m_xspeedLimiter.calculate(dj.getLeftY()) * Constants.drivetrain.kMaxSpeed;
    final var ySpeed = -m_yspeedLimiter.calculate(dj.getLeftX()) * Constants.drivetrain.kMaxSpeed;
    final var rot = -m_rotLimiter.calculate(dj.getRightX()) * Constants.drivetrain.kMaxAngularSpeed;
    m_drive.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
