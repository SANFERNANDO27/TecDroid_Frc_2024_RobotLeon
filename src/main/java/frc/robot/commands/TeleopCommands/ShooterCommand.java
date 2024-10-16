package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ShooterPositionerConstants;
import frc.robot.constants.VelocityConstants;
import frc.robot.subsystems.Sensors.LimitSwitches;
import frc.robot.subsystems.basic.Indexer;
import frc.robot.subsystems.basic.Intake;
import frc.robot.subsystems.basic.Shooter;
import frc.robot.subsystems.basic.ShooterPositioner;

import java.util.function.Supplier;

public class ShooterCommand extends Command {

    private final Shooter shooter;
    private final ShooterPositioner shooterPositioner;
    private final Intake intake;
    private final Indexer indexer;
    private final Supplier<Double> rightTriggerAxis, leftTriggerAxis;
    private Supplier<Boolean> neutralTrigger, speakerButton, ampButton, lowPassButton, longPassButton, climberButton;
    private boolean neutralState, speakerState, ampState, lowPassState, longPassState, climberState;

    public ShooterCommand(Supplier<Double> rightTriggerAxis, Supplier<Double> leftTriggerAxis, 
            Supplier<Boolean> neutralTigger, Supplier<Boolean> speakerButton, Supplier<Boolean> ampButton,
            Supplier<Boolean> lowPassButton, Supplier<Boolean> longPassButton, Supplier<Boolean> climberButton, 
            Shooter shooter, ShooterPositioner shooterPositioner, Intake intake, Indexer indexer) {
        this.shooterPositioner = shooterPositioner;
        this.shooter = shooter;
        this.intake = intake;
        this.indexer = indexer;
        this.rightTriggerAxis = rightTriggerAxis;
        this.leftTriggerAxis = leftTriggerAxis;

        // Buttons
        this.neutralTrigger = neutralTigger;
        this.speakerButton = speakerButton;
        this.ampButton = ampButton;
        this.lowPassButton = lowPassButton;
        this.longPassButton = longPassButton;
        this.climberButton = climberButton;

        // states
        this.neutralState = true;
        this.speakerState = false;
        this.ampState = false;
        this.lowPassState = false;
        this.longPassState = false;
        this.climberState = false;

    }

    @Override
    public void initialize() {}

    public void changeState() {
        if (!neutralTrigger.get()) {
            neutralState = true;
            speakerState = false;
            ampState = false;
            lowPassState = false;
            longPassState = false;
        }else if (speakerButton.get()) {
            neutralState = false;
            speakerState = true;
            ampState = false;
            lowPassState = false;
            longPassState = false;
        } else if(ampButton.get()) {
            neutralState = false;
            speakerState = false;
            ampState = true;
            lowPassState = false;
            longPassState = false;
        } else if(lowPassButton.get()) {
            neutralState = false;
            speakerState = false;
            ampState = false;
            lowPassState = true;
            longPassState = false;
        } else if(longPassButton.get()) {
            neutralState = false;
            speakerState = false;
            ampState = false;
            lowPassState = false;
            longPassState = true;
        }
    }

    public void setVelocity(double rpmVelocity) {
        shooter.setVelocity(rpmVelocity, rpmVelocity);
    }

    public boolean shootIsPressed() {
        return rightTriggerAxis.get() > 0.0;
    }

    public void intakeAndIndexerMove() {
        if (shooter.isAtSetPoint(Constants.ShooterConstants.shootVelocity) && shooterPositioner.isAtSetPoint(shooterPositioner.getEstimatedShooterPositionerAngleWithRect())) {
            // Move intake and indexer when the shooter it's at set point
            indexer.setVelocity(2000);
            intake.setVelocity(2000);
        } else {
            indexer.setPercentage(0);
            intake.setPercentage(0);
        }
    }

    public void shootAtVelocity(double velocity) {
        if (shootIsPressed()) {
            setVelocity(velocity);
        }else {
            shooterPositioner.setPercentage(0.0, 0.0);
        }
    }

    public void moveShooterPositionerAtPosition(double position) {
        if (shootIsPressed()) {
            shooterPositioner.goToPosition(position);
        }else {
            shooterPositioner.setPercentage(0.0, 0.0);
        }
    }

    // Neutral State 
    public void moveShooterPositionerAtNeutralPosition() {
        shooterPositioner.goToPosition(ShooterPositionerConstants.neutralPosition);
    }

    // Speaker state
    public void shootAtSpeaker() {
        double shooterPercentage = rightTriggerAxis.get() - leftTriggerAxis.get();
        double rpms = ShooterConstants.shootVelocity * shooterPercentage;
        setVelocity(rpms);
    }

    public void moveShooterPositionerAtSpeaker() {
        if (shootIsPressed()) {
                shooterPositioner.goToEstimatedShooterPositionAngle();
        }else {
            shooterPositioner.setPercentage(0.0, 0.0);
        }
    }


    @Override
    public void execute() {
        changeState();
        
        // In case that there's not a note
        if (neutralState) {
            // Shooter angle
            moveShooterPositionerAtNeutralPosition();
            if (neutralTrigger.get()) {
                speakerState = true;
                neutralState = false;
            }
        }else if (speakerState) { 
            // Shooter
            shootAtSpeaker();

            // Shooter angle
            moveShooterPositionerAtSpeaker();

            // Intake and indexer
            intakeAndIndexerMove();
        }else if (ampState) {
            // Shooter
            shootAtVelocity(VelocityConstants.Velocities.kAmpShooterVelocity);

            // Shooter angle
            moveShooterPositionerAtPosition(Constants.ShooterPositionerConstants.ampPosition);

            // Intake and indexer
            intakeAndIndexerMove();
        } else if (lowPassState) {
            // Shooter
            shootAtVelocity(VelocityConstants.Velocities.kLowPassShooterVelocity);

            // Shooter angle
            moveShooterPositionerAtPosition(Constants.ShooterPositionerConstants.lowPassPosition);

            // Intake and indexer
            intakeAndIndexerMove();
        } else if (longPassState) {
            // Shooter
            shootAtVelocity(VelocityConstants.Velocities.kLongPassShooterVelocity);

            // Shooter angle
            moveShooterPositionerAtPosition(Constants.ShooterPositionerConstants.longPassPosition);

            // Intake and indexer
            intakeAndIndexerMove();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}