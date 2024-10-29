package frc.robot.subsystems.basic;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants.IntakeConstants;


public class Intake {
    
    // Top
    private final CANSparkMax topMotor;
    private final RelativeEncoder topEncoder;
    private final SparkPIDController topPidController;

    // Bottom
    private final CANSparkMax bottomMotor;
    private final RelativeEncoder bottomEncoder;
    private final SparkPIDController bottomPidController;
    

    public Intake(final int topId, final int bottomId) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Intake");
        shooterTab.addDouble("Top (RPMs)", () -> {return getTopVelocity();});
        shooterTab.addDouble("Bottom (RPMs)", () -> {return getBottomVelocity();});

        topMotor = new CANSparkMax(topId, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(bottomId, MotorType.kBrushless);
        topMotor.setInverted(true);
        bottomMotor.setInverted(false);

        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();

        topEncoder.setPositionConversionFactor(1);
        topEncoder.setVelocityConversionFactor(1);
        bottomEncoder.setPositionConversionFactor(1);
        bottomEncoder.setVelocityConversionFactor(1);

        // PID
        topPidController = topMotor.getPIDController();
        topPidController.setP(IntakeConstants.topMotor_P);
        topPidController.setI(IntakeConstants.topMotor_I);
        topPidController.setD(IntakeConstants.topMotor_D);
        topPidController.setFF(IntakeConstants.topMotor_f);

        bottomPidController = topMotor.getPIDController();
        bottomPidController.setP(IntakeConstants.bottomMotor_P);
        bottomPidController.setI(IntakeConstants.bottomMotor_I);
        bottomPidController.setD(IntakeConstants.bottomMotor_D);
        bottomPidController.setFF(IntakeConstants.bottomMotor_f);

    }

    public double getTopVelocity() {
        return topEncoder.getVelocity();
    }
    
    public double getBottomVelocity() {
        return bottomEncoder.getVelocity();
    }

    public void setPercentage(final double motorPercentage) {
        topMotor.set(motorPercentage);
        bottomMotor.set(motorPercentage);

    }

    public void setVelocity(final double rpms) {
        topPidController.setReference(rpms, ControlType.kVelocity);
        bottomPidController.setReference(rpms, ControlType.kVelocity);

    }


}
