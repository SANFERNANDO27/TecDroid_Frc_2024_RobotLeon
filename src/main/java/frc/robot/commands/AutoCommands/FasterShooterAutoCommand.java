package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.subsystems.Sensors.LimitSwitches;
import frc.robot.subsystems.basic.Indexer;
import frc.robot.subsystems.basic.Intake;
import frc.robot.subsystems.basic.Shooter;
import frc.robot.subsystems.basic.ShooterPositioner;
import frc.robot.subsystems.drivetrain.SwerveAlignToAprilTag;

public class FasterShooterAutoCommand extends Command {

    private final Shooter shooter;

    public FasterShooterAutoCommand(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.setVelocity(ShooterConstants.autoShootVelocity, ShooterConstants.autoShootVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPercentage(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}