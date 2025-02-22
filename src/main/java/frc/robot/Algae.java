package frc.robot;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;

import com.revrobotics.spark.config.SparkMaxConfig;

public class Algae {
    private final SparkMax flipperLeft = new SparkMax(Constants.algae.flipper.left.kMotorId, MotorType.kBrushless);
    private final SparkMax flipperRight = new SparkMax(Constants.algae.flipper.right.kMotorId, MotorType.kBrushless);
    private final SparkClosedLoopController flipperController = flipperLeft.getClosedLoopController();
    private final SparkMax intakeTop = new SparkMax(Constants.algae.intake.top.kMotorId, MotorType.kBrushless);
    private final SparkMax intakeBottom = new SparkMax(Constants.algae.intake.bottom.kMotorId, MotorType.kBrushless);

    public Algae() {
        SparkMaxConfig defaultConfig = new SparkMaxConfig();
        SparkMaxConfig flConfig = new SparkMaxConfig();
        SparkMaxConfig frConfig = new SparkMaxConfig();
        SparkMaxConfig itConfig = new SparkMaxConfig();
        SparkMaxConfig ibConfig = new SparkMaxConfig();
        defaultConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
        flConfig.apply(defaultConfig)
            .inverted(Constants.algae.flipper.left.kInverted);
        flipperLeft.configure(flConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frConfig.apply(defaultConfig)
            .follow(flipperLeft, Constants.algae.flipper.right.kInverted);
        flipperRight.configure(frConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        itConfig.apply(defaultConfig)
            .smartCurrentLimit(20)
            .inverted(Constants.algae.intake.top.kInverted);
        intakeTop.configure(itConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ibConfig.apply(defaultConfig)
            .smartCurrentLimit(20)
            .follow(intakeTop, Constants.algae.intake.bottom.kInverted);
        intakeBottom.configure(ibConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void intake() {
        //intakeBottom follows intakeTop
        intakeTop.set(Constants.algae.intake.kInSpeed);
    }

    public void stop() {
        //intakeBottom follows intakeTop
        intakeTop.set(0);
    }

    public void outtake() {
        //intakeBottom follows intakeTop
        intakeTop.set(-Constants.algae.intake.kOutSpeed);
    }

    public void flipperforward() {
        //flipperRight follows flipperLeft
        flipperSetPosition(Constants.algae.flipper.kForwardPosition);
    }

    public void flipperbackward() {
        //flipperRight follows flipperLeft
        flipperSetPosition(Constants.algae.flipper.kBackwardPosition);
    }

    private void flipperSetPosition(double position) {
        flipperController.setReference(position, ControlType.kPosition);
    }
}
