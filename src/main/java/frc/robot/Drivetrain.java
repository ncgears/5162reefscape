// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/** Represents a mecanum drive style drivetrain. */
public class Drivetrain {
  private final SparkMax frontLeft = new SparkMax(Constants.drivetrain.frontleft.kMotorId, MotorType.kBrushless);
  private final SparkMax backLeft = new SparkMax(Constants.drivetrain.backleft.kMotorId, MotorType.kBrushless);
  private final SparkMax frontRight = new SparkMax(Constants.drivetrain.frontright.kMotorId, MotorType.kBrushless);
  private final SparkMax backRight = new SparkMax(Constants.drivetrain.backright.kMotorId, MotorType.kBrushless);

  private final Encoder m_frontLeftEncoder = new Encoder(0, 1);
  private final Encoder m_frontRightEncoder = new Encoder(2, 3);
  private final Encoder m_backLeftEncoder = new Encoder(4, 5);
  private final Encoder m_backRightEncoder = new Encoder(6, 7);

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final PIDController m_frontLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_frontRightPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backRightPIDController = new PIDController(1, 0, 0);

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(m_kinematics, m_gyro.getRotation2d(), getCurrentDistances());

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /** Constructs a MecanumDrive and resets the gyro. */
  public Drivetrain() {
    // Setup the SparkMax for drivetrain
    SparkMaxConfig defaultConfig = new SparkMaxConfig();
    SparkMaxConfig flConfig = new SparkMaxConfig();
    SparkMaxConfig frConfig = new SparkMaxConfig();
    SparkMaxConfig blConfig = new SparkMaxConfig();
    SparkMaxConfig brConfig = new SparkMaxConfig();
    defaultConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
    flConfig.apply(defaultConfig).inverted(Constants.drivetrain.frontleft.kInverted);
    frConfig.apply(defaultConfig).inverted(Constants.drivetrain.frontright.kInverted);
    blConfig.apply(defaultConfig).inverted(Constants.drivetrain.backleft.kInverted);
    brConfig.apply(defaultConfig).inverted(Constants.drivetrain.backright.kInverted);
    frontLeft.configure(flConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRight.configure(frConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeft.configure(blConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRight.configure(brConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // m_drive = new MecanumDrive(frontLeft::set, backLeft::set, frontRight::set, backRight::set);

    m_gyro.reset();
  }

  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_backLeftEncoder.getRate(),
        m_backRightEncoder.getRate());
  }

  /**
   * Returns the current distances measured by the drivetrain.
   *
   * @return The current distances measured by the drivetrain.
   */
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_backLeftEncoder.getDistance(),
        m_backRightEncoder.getDistance());
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput =
        m_frontLeftPIDController.calculate(
            m_frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        m_frontRightPIDController.calculate(
            m_frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        m_backLeftPIDController.calculate(
            m_backLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        m_backRightPIDController.calculate(
            m_backRightEncoder.getRate(), speeds.rearRightMetersPerSecond);

    frontLeft.setVoltage(frontLeftOutput + frontLeftFeedforward);
    frontRight.setVoltage(frontRightOutput + frontRightFeedforward);
    backLeft.setVoltage(backLeftOutput + backLeftFeedforward);
    backRight.setVoltage(backRightOutput + backRightFeedforward);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    mecanumDriveWheelSpeeds.desaturate(Constants.drivetrain.kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), getCurrentDistances());
  }
}