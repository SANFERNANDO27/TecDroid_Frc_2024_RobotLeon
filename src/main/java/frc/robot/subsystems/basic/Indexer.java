package frc.robot.subsystems.basic;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;


public class Indexer {
    
    private final TalonFX indexerMotor;

    private final Slot0Configs indexerPidController;

    public Indexer(final int motorId) {
        ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");
        indexerTab.addDouble("Velocity (RPMs)", () -> {return getMotorVelocity();});

        indexerMotor = new TalonFX(motorId);
        indexerMotor.setInverted(true);

        // PID
        indexerPidController = new Slot0Configs();
        indexerPidController.kS = 0.1; // Add 0.1 V output to overcome static friction
        indexerPidController.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        indexerPidController.kP = IndexerConstants.motor_P; // An error of 1 rps results in 0.11 V output
        indexerPidController.kI = IndexerConstants.motor_I; // no output for integrated error
        indexerPidController.kD = IndexerConstants.motor_D; // no output for error derivative
        indexerMotor.getConfigurator().apply(indexerPidController);

    }

    public double getMotorVelocity() {
        return indexerMotor.getVelocity().getValueAsDouble();
    }

    public void setPercentage(final double motorPercentage) {
        indexerMotor.set(motorPercentage);

    }

    public void setVelocity(final double rpms) {
        // create a velocity closed-loop request, voltage output, slot 0 configs
        VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        indexerMotor.setControl(m_request.withVelocity(rpms).withFeedForward(IndexerConstants.motor_f));

    }


}
