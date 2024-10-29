package frc.robot.subsystems.basic;

import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants.ShooterPositionerConstants;
import frc.robot.constants.ShooterPoseObject;
import frc.robot.subsystems.Sensors.Limelight;
import frc.robot.subsystems.Sensors.LimitSwitches;


public class ShooterPositioner {
    
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;
    private final DutyCycleEncoder positionerEncoder;

    private final SparkPIDController rightPidController;
    private final SparkPIDController leftPidController;
    private final PIDController pidController;
    
    private final LimitSwitches limitSwitches;

    private final Limelight limelight = new Limelight();

    List<ShooterPoseObject<Double, Double, Double>> estimatedShooterPoseList;

    public ShooterPositioner(final int rightId, final int leftId, LimitSwitches limitSwitches) {
        rightMotor = new CANSparkMax(rightId, MotorType.kBrushless);
        leftMotor = new CANSparkMax(leftId, MotorType.kBrushless);

        // Positioner encoder
        positionerEncoder = new DutyCycleEncoder(0);

        // Invert Bottom motor
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);

        // Idle Brake mode
        // Set Idle Mode to brake to avoid shooter positioner falling
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        rightEncoder.setPositionConversionFactor(1);
        leftEncoder.setPositionConversionFactor(1);
        rightEncoder.setVelocityConversionFactor(1);
        leftEncoder.setVelocityConversionFactor(1);

        // PID's
        rightPidController = rightMotor.getPIDController();
        rightPidController.setP(ShooterPositionerConstants.rightP);
        rightPidController.setI(ShooterPositionerConstants.rightI);
        rightPidController.setD(ShooterPositionerConstants.rightD);
        rightPidController.setFF(ShooterPositionerConstants.rightF);

        leftPidController = leftMotor.getPIDController();
        leftPidController.setP(ShooterPositionerConstants.leftP);
        leftPidController.setI(ShooterPositionerConstants.leftI);
        leftPidController.setD(ShooterPositionerConstants.leftD);
        leftPidController.setFF(ShooterPositionerConstants.leftF);

        ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterPositioner");

        shooterTab.addDouble("Right Velocity (RPMs)", () -> {return getRightVelocity();});
        shooterTab.addDouble("Left Velocity (RPMs)", () -> {return getLeftVelocity();});
        shooterTab.addDouble("Encoder distance", () -> {return getEncoderPositionInDegrees();});
        shooterTab.addBoolean("LimitSwitch Read", () -> {return limitSwitches.getShooterPositionerLimitSwitch();});

        shooterTab.addDouble("Limelight X: ", () -> {return limelight.getX();});
        shooterTab.addDouble("Estimated Pose: ", () -> {return getEstimatedShooterPositionerAngleWithRect();});

        pidController = new PIDController(
            ShooterPositionerConstants.kP, 
            ShooterPositionerConstants.kI, 
            ShooterPositionerConstants.kD);
        pidController.setTolerance(2, 10);

        this.limitSwitches = limitSwitches;
    }

    public double getEstimatedShooterPositionerAngleWithRect() {
        double limelightY = limelight.getY();

        if (limelightY > 7) {
            return -(4.88/4.28) * (limelight.getY()) + 30;
        } else {
            return -(4.88/4.28) * (limelight.getY()) + 34;
        }
        
    }

    public void goToEstimatedShooterPositionAngle() {
        goToPosition(getEstimatedShooterPositionerAngleWithRect());

    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getEncoderPositionInRot() {
        return positionerEncoder.getAbsolutePosition();
    }

    public double getEncoderPositionInDegrees() {
        double positionEncoderInDegrees = getEncoderPositionInRot() * 360;
        // set 0 position
        positionEncoderInDegrees -= ShooterPositionerConstants.zeroPosition;
        return positionEncoderInDegrees;
    }

    public void setPercentage(final double leftPercentage, final double rightPercentage) {
        // set limits
        if ((getEncoderPositionInDegrees() < ShooterPositionerConstants.downLimitInDegrees || limitSwitches.getShooterPositionerLimitSwitch()) && leftPercentage < 0 || getEncoderPositionInDegrees() > ShooterPositionerConstants.upLimitInDegrees && leftPercentage > 0) {
            rightMotor.set(0.0);
            leftMotor.set(0.0);
        }else {
            rightMotor.set(rightPercentage);
            leftMotor.set(leftPercentage);
        }
    }

    public void setRawPercentage(final double leftPercentage, final double rightPercentage) {
        rightMotor.set(rightPercentage);
        leftMotor.set(leftPercentage);
    }

    public void setVelocity(final double leftRpms, final double rightRpms) {
        rightPidController.setReference(rightRpms, ControlType.kVelocity);
        leftPidController.setReference(leftRpms, ControlType.kVelocity);

    }

    public boolean isAtSetPoint(double setPoint) {
        if (setPoint + ShooterPositionerConstants.setPointTolerance > getEncoderPositionInDegrees() && getEncoderPositionInDegrees() > setPoint - ShooterPositionerConstants.setPointTolerance) {
            return true;
        }else {
            return false;
        }
    }

    public double goToPosition(double setPoint) {
        // PID calculation
        double percentage = pidController.calculate(getEncoderPositionInDegrees(), setPoint);

        // Positioner limits
        if ((getEncoderPositionInDegrees() < ShooterPositionerConstants.downLimitInDegrees || limitSwitches.getShooterPositionerLimitSwitch()) && percentage < 0 || getEncoderPositionInDegrees() > ShooterPositionerConstants.upLimitInDegrees && percentage > 0) {
            rightMotor.set(0.0);
            leftMotor.set(0.0);
        }else {
            rightMotor.set(percentage);
            leftMotor.set(percentage);
        }

        return percentage;
    }

    public void resetEncoder() {
        positionerEncoder.reset();
    }


}
