
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands.FasterShooterAutoCommand;
import frc.robot.commands.AutoCommands.IntakeAndAutoIndexerCommand;
import frc.robot.commands.AutoCommands.ShooterAutoCommand;
import frc.robot.commands.AutoCommands.StartShoot;
import frc.robot.commands.TeleopCommands.SwerveJoystickCmd;
import frc.robot.commands.TeleopCommands.IntakeAndIndexerCommand;
import frc.robot.commands.TeleopCommands.TeleopStatesCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.SwerveConstants.OIConstants;
import frc.robot.subsystems.Sensors.LimitSwitches;
import frc.robot.subsystems.basic.Climber;
import frc.robot.subsystems.basic.Indexer;
import frc.robot.subsystems.basic.Intake;
import frc.robot.subsystems.basic.Shooter;
import frc.robot.subsystems.basic.ShooterPositioner;
import frc.robot.subsystems.drivetrain.SwerveAlignToAprilTag;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

  private final Shooter shooter = new Shooter(Constants.ShooterConstants.BOTTOM_SHOOTER_ID, Constants.ShooterConstants.Top_SHOOTER_ID);
  private final LimitSwitches limitSwitches = new LimitSwitches();
  private final ShooterPositioner shooterPositioner = new ShooterPositioner(Constants.ShooterPositionerConstants.rightPositionerId, Constants.ShooterPositionerConstants.leftPositionerId, limitSwitches);
  private final Intake intake = new Intake(Constants.IntakeConstants.topMotorId, Constants.IntakeConstants.bottomMotorId);
  private final Indexer indexer = new Indexer(Constants.IndexerConstants.motorId);
  private final Climber climber = new Climber(Constants.ClimberConstants.rightMotorId, Constants.ClimberConstants.leftMotorId);
  private final ShuffleboardTab robotContainerTab;
  private final XboxController controller = new XboxController(0);

  // Auto
  private final SwerveAlignToAprilTag swerveAlignToAprilTag;
  private final IntakeAndAutoIndexerCommand intakeAndAutoIndexerCommand;
  private final ShooterAutoCommand shooterAutoCommand;
  private final FasterShooterAutoCommand fasterShooterAutoCommand;

  // Commands
  IntakeAndIndexerCommand intakeAndIndexerCommand = new IntakeAndIndexerCommand(indexer, intake, limitSwitches);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> true,
                () -> controller.getRightTriggerAxis() > 0.0));
          
        climber.setDefaultCommand(new TeleopStatesCommand(
              climber,
              shooter,
              shooterPositioner,
              intake,
              indexer,
              () -> controller.getRightTriggerAxis(),
              () -> controller.getLeftTriggerAxis(),
              () -> controller.getRightStickButton(),
              () -> controller.getAButton(),
              () -> controller.getYButton(),
              () -> controller.getBButton(),
              () -> controller.getXButton(),
              () -> false));
              
        // !!!! Auto !!!!

        // Auto commands and susbsystems
        swerveAlignToAprilTag = new SwerveAlignToAprilTag(swerveSubsystem);
        intakeAndAutoIndexerCommand = new IntakeAndAutoIndexerCommand(indexer, intake, shooter, limitSwitches);
        shooterAutoCommand = new ShooterAutoCommand(indexer, intake, shooter, limitSwitches, swerveAlignToAprilTag, shooterPositioner);
        fasterShooterAutoCommand = new FasterShooterAutoCommand(shooter);

        // Pathplanner named commands
        NamedCommands.registerCommand("IntakeAndIndexerCommand", intakeAndAutoIndexerCommand);
        NamedCommands.registerCommand("Shooter", shooterAutoCommand);
        NamedCommands.registerCommand("FasterShooter", fasterShooterAutoCommand);
        NamedCommands.registerCommand("StartShoot", new StartShoot(indexer, intake, shooter, limitSwitches, swerveAlignToAprilTag, shooterPositioner));

        // Shuffle board
        robotContainerTab = Shuffleboard.getTab("Robot Container");
        robotContainerTab.addBoolean("Limit Switches Read: ", () -> {return limitSwitches.getShooterLimitSwitch();});
        robotContainerTab.addBoolean("Shooter is at set point: ", () -> {return shooter.isAtSetPoint(Constants.ShooterConstants.shootVelocity);});
        robotContainerTab.addDouble("X robot position: ", () -> {return swerveSubsystem.getPose().getX();});
        robotContainerTab.addDouble("Y robot position: ", () -> {return swerveSubsystem.getPose().getY();});
        robotContainerTab.addDouble("Theta robot position: ", () -> {return swerveSubsystem.getPose().getRotation().getDegrees();});
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reset gyro
        new JoystickButton(controller, XboxController.Button.kStart.value).onTrue(swerveSubsystem.runOnce(swerveSubsystem::zeroHeading));
        /*

        // Climber
        new Trigger(() -> (controller.getPOV() == 0) || (controller.getPOV() == 180))
          .whileTrue(new RunCommand(() -> climber.setVelocity(ClimberConstants.velocity * ((controller.getPOV() == 180) ? -1.0 : 1.0), ClimberConstants.velocity * ((controller.getPOV() == 180) ? -1.0 : 1.0))))
          .onFalse(new InstantCommand(() -> climber.setPercentage(0.0, 0.0)));
        */
        
        // Intake && indexer
        new Trigger(() -> controller.getLeftBumper() || controller.getRightBumper() && !limitSwitches.getShooterLimitSwitch())
          .whileTrue(new RunCommand(() -> intakeAndIndexerCommand.setIntakeAndIndexer(0.5 * (controller.getLeftBumper() ? -1.0 : 1.0),
                                                                                        0.5 * (controller.getLeftBumper() ? -1.0 : 1.0))))
          .onFalse(new InstantCommand(() -> intakeAndIndexerCommand.stopIntakeAndIndexer()));
        
        // ShooterPositioner
        new Trigger(() -> (controller.getPOV() == 270) || (controller.getPOV() == 90))
          .whileTrue(new RunCommand(() -> shooterPositioner.setPercentage(0.1 * ((controller.getPOV() == 270) ? -1.0 : 1.0), 0.1 * ((controller.getPOV() == 270) ? -1.0 : 1.0))))
          .onFalse(new InstantCommand(() -> shooterPositioner.setPercentage(0.0, 0.0)));
    }

    public Command getAutonomousCommand() {
        // PathPlanner Auto
        return new PathPlannerAuto("FasterNotesAuto");
    }
  
  public void teleopPeriodic() {
  }
}
