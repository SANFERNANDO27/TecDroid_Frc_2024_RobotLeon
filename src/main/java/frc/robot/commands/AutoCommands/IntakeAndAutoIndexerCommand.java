package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.subsystems.Sensors.LimitSwitches;
import frc.robot.subsystems.basic.Indexer;
import frc.robot.subsystems.basic.Intake;
import frc.robot.subsystems.basic.Shooter;

public class IntakeAndAutoIndexerCommand extends Command {

    private final Indexer indexer;
    private final Intake intake;
    private final LimitSwitches limitSwitches;

    public IntakeAndAutoIndexerCommand(Indexer indexer, Intake intake, Shooter shooter, LimitSwitches limitSwitches) {
        this.indexer = indexer;
        this.intake = intake;
        this.limitSwitches = limitSwitches;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!limitSwitches.getShooterLimitSwitch()) {
            indexer.setVelocity(IndexerConstants.AutoVelocity);
            intake.setVelocity(IndexerConstants.AutoVelocity);
        } else {
            indexer.setVelocity(0);
            intake.setVelocity(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercentage(0);
        intake.setPercentage(0);
    }

    @Override
    public boolean isFinished() {
        return limitSwitches.getShooterLimitSwitch();
    }
}