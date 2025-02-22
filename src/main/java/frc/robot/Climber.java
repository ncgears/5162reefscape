package frc.robot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber {
    private final SparkMax climber = new SparkMax(Constants.climber.kMotorId, MotorType.kBrushless);
    private final SparkClosedLoopController climberController = climber.getClosedLoopController();

    public Climber() {
        SparkMaxConfig defaultConfig = new SparkMaxConfig();
        SparkMaxConfig cConfig = new SparkMaxConfig();
        defaultConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        //climber
        cConfig.apply(defaultConfig)
            .inverted(Constants.algae.flipper.left.kInverted);

        climber.configure(cConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climbup() {
        climber.set(Constants.climber.kSpeed);
    }

    public void stop() {
        climber.set(0);
    }

    public void climbdown() {
        climber.set(-Constants.climber.kSpeed);
    }

    private void climberSetPosition(double position) {
        climberController.setReference(position, ControlType.kPosition);
    }
}
