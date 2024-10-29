// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;
import frc.robot.constants.ShooterPoseObject;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ShooterConstants {
    public static final int BOTTOM_SHOOTER_ID = 54;
    public static final int Top_SHOOTER_ID = 53;

    // PID's
    public static final double BOTTOM_P = 0.00015;
    public static final double BOTTOM_I = 0.0;
    public static final double BOTTOM_D = 0.0005;
    public static final double BOTTOM_F = 0.000225;

    public static final double TOP_P = 0.0004;
    public static final double TOP_I = 0.0;
    public static final double TOP_D = 0.0;
    public static final double TOP_F = 0.000225;

    public static final double shootVelocity = 3000/60;

    // Auto
    public static final double autoShootVelocity = 416;

    // Tolerance
    public static final double setpointRpmsTolerance = 3.0;
    
  }

  public static class ShooterPositionerConstants {
    // Id's
    public static final int leftPositionerId = 51;
    public static final int rightPositionerId = 52;

    // Shooter positioner motors velocity
    public static final double shooterPositionerVelocity = 700;

    // PID's

    // Motors PID's
    public static final double rightP = 0.00005;
    public static final double rightI = 0.0;
    public static final double rightD = 0.0005;
    public static final double rightF = 0.000225;

    public static final double leftP = 0.00015;
    public static final double leftI = 0.0;
    public static final double leftD = 0.0;
    public static final double leftF = 0.000225;

    // Positioner encoder PID
    public static final double kP = 0.025;
    public static final double kI = 0.00001;
    public static final double kD = 0.0;
    public static final double setPointTolerance = 1.0;


    // Shooter Positions
    public static final double neutralPosition = 2;
    public static final double ampPosition = 125.8;
    public static final double trapPosition = 0.5;
    public static final double lowPassPosition = 40;
    public static final double longPassPosition = 0.5;

    // Zero position 
    public static final double zeroPosition = 99.3;

    // Shooter limits
    public static final double downLimitInDegrees = 0.0;
    public static final double upLimitInDegrees = 200.0;

  }

  public static class IntakeConstants {
    public static final int topMotorId = 1;
    public static final int bottomMotorId = 2;

    public static final int maxVelocity = 5000;
    public static final double CONFIGURED_SETPOINT = 1500;

    // !!!! Top !!!!

    // PID's
    public static final double topMotor_P = 0.00005;
    public static final double topMotor_I = 0.0;
    public static final double topMotor_D = 0.0;
    public static final double topMotor_f = 0.000225;

    // !!!! Bottom !!!!

    // PID's
    public static final double bottomMotor_P = 0.00005;
    public static final double bottomMotor_I = 0.0;
    public static final double bottomMotor_D = 0.0;
    public static final double bottomMotor_f = 0.000225;

    // Auto
    public static final double AutoVelocity = 2700;
  }

  public static class IndexerConstants {
    public static final int motorId = 50;

    public static final int maxVelocity = 5000;

    // PID's
    public static final double motor_P = 0.00005;
    public static final double motor_I = 0.0;
    public static final double motor_D = 0.0;
    public static final double motor_f = 0.000225;

    public static final double CONFIGURED_SETPOINT = 700;

    // Auto
    public static final double AutoVelocity = 2700;
  }

  public static class ClimberConstants {
    public static final int rightMotorId = 62;
    public static final int leftMotorId = 61;

    // Motors PID's
    public static final double rightP = 0.00005;
    public static final double rightI = 0.0;
    public static final double rightD = 0.0005;
    public static final double rightF = 0.000225;

    public static final double leftP = 0.00015;
    public static final double leftI = 0.0;
    public static final double leftD = 0.0;
    public static final double leftF = 0.000225;

    public static final double velocity = 2000;

    // Limits
    public static final double upLimit = 255.0;
    public static final double downLimit = 0.0;

  }

  public static class LimitSwitchConstants {
    public static final int shooterLimitSwitchId = 1;
    public static final int shooterPositionerLimitSwitchId = 2;
    
  }

  public static class GameStrategiesConstants {
    public static final boolean limitSwitchIntakeIndexerShooter = false;
    
  }
}
