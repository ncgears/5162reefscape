// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private final Drivetrain m_drive;
  private final XboxController dj = new XboxController(Constants.oi.kDriverId);
  private final XboxController oj = new XboxController(Constants.oi.kOperId);

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_drive = new Drivetrain();
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_drive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    final var xSpeed = -m_xspeedLimiter.calculate(dj.getLeftY()) * Constants.drivetrain.kMaxSpeed;
    final var ySpeed = -m_yspeedLimiter.calculate(dj.getLeftX()) * Constants.drivetrain.kMaxSpeed;
    final var rot = -m_rotLimiter.calculate(dj.getRightX()) * Constants.drivetrain.kMaxAngularSpeed;
    m_drive.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
