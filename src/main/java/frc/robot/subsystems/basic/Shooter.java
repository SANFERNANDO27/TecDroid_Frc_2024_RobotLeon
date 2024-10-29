package frc.robot.subsystems.basic;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.subsystems.Sensors.Limelight;
import frc.robot.constants.ShooterPoseObject;


public class Shooter extends SubsystemBase{
    
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    private final Slot0Configs bottomPidController;
    private final Slot0Configs topPidController;

    private final Limelight limelight = new Limelight();
    private final List<ShooterPoseObject<Double, Double, Double>> estimatedShooterVelocityList;

    public Shooter(final int bottomId, final int topId) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
        shooterTab.addDouble("Bottom Velocity (RPMs)", () -> {return getBottomVelocity();});
        shooterTab.addDouble("Top Velocity (RPMs)", () -> {return getTopVelocity();});

        topMotor = new TalonFX(topId);
        bottomMotor = new TalonFX(bottomId);

        // Invert Bottom motor
        bottomMotor.setInverted(true);

        // PID's https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
        bottomPidController = new Slot0Configs();
        bottomPidController.kS = 0.1; // Add 0.1 V output to overcome static friction
        bottomPidController.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        bottomPidController.kP = ShooterConstants.BOTTOM_P; // An error of 1 rps results in 0.11 V output
        bottomPidController.kI = ShooterConstants.BOTTOM_I; // no output for integrated error
        bottomPidController.kD = ShooterConstants.BOTTOM_D; // no output for error derivative
        bottomMotor.getConfigurator().apply(bottomPidController);

        topPidController = new Slot0Configs();
        topPidController.kS = 0.1; // Add 0.1 V output to overcome static friction
        topPidController.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        topPidController.kP = ShooterConstants.TOP_P; // An error of 1 rps results in 0.11 V output
        topPidController.kI = ShooterConstants.TOP_I; // no output for integrated error
        topPidController.kD = ShooterConstants.TOP_D; // no output for error derivative
        topMotor.getConfigurator().apply(topPidController);

        // Estimated velocity acording to the pose range
        estimatedShooterVelocityList = new ArrayList<>();
            estimatedShooterVelocityList.add(new ShooterPoseObject<>(1.0, 6.0, 1500.0));
            estimatedShooterVelocityList.add(new ShooterPoseObject<>(6.0, 12.5, 2000.0));
            estimatedShooterVelocityList.add(new ShooterPoseObject<>(12.5, 20.0, 2000.0));
    }

    public double getBottomVelocity() {
        return bottomMotor.getVelocity().getValueAsDouble();
    }

    public double getTopVelocity() {
        return topMotor.getVelocity().getValueAsDouble();
    }

    public void setPercentage(final double topPercentage, final double bottomPercentage) {
        bottomMotor.set(bottomPercentage);
        topMotor.set(topPercentage);

    }

    public boolean isAtSetPoint(double setPointRpms) {
        if (getTopVelocity() > setPointRpms - ShooterConstants.setpointRpmsTolerance
            && getBottomVelocity() > setPointRpms - ShooterConstants.setpointRpmsTolerance) {
            return true;
        }else {
            return false;
        }
    }

    public void setVelocity(final double topRps, final double bottomRps) {
        // create a velocity closed-loop request, voltage output, slot 0 configs
        VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        topMotor.setControl(m_request.withVelocity(topRps).withFeedForward(ShooterConstants.TOP_F));
        bottomMotor.setControl(m_request.withVelocity(bottomRps).withFeedForward(ShooterConstants.BOTTOM_F));

    }

    public double getEstimatedShooterVelocity() {
        double velocity = 2000;

        // Compare all the posible distances with a structure
        for (ShooterPoseObject<Double, Double, Double> item : estimatedShooterVelocityList) {
            // invert the limelight read to compare it
            if (-limelight.getY() < item.getMaxDistance() && -limelight.getY() > item.getMinDistance()) {
                velocity = item.getEstimatedAngle().doubleValue();
            }
        }

        return velocity;
    }

}
