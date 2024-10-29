package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ShooterPositionerConstants;
import frc.robot.constants.VelocityConstants;
import frc.robot.subsystems.Sensors.LimitSwitches;
import frc.robot.subsystems.basic.Climber;
import frc.robot.subsystems.basic.Indexer;
import frc.robot.subsystems.basic.Intake;
import frc.robot.subsystems.basic.Shooter;
import frc.robot.subsystems.basic.ShooterPositioner;

import java.util.function.Supplier;

public class TeleopStatesCommand extends Command {

    private final Shooter shooter;
    private final ShooterPositioner shooterPositioner;
    private final Indexer indexer;
    private final Climber climber;
    private final Supplier<Double> rightTriggerAxis, leftTriggerAxis;
    private Supplier<Boolean> neutralTrigger, speakerButton, ampButton, lowPassButton, longPassButton, climberButton;
    private boolean neutralState, speakerState, ampState, lowPassState, longPassState, climberState;

    double climbStages = 0;

    public TeleopStatesCommand(Climber climber, Shooter shooter, ShooterPositioner shooterPositioner, Intake intake, Indexer indexer,
            Supplier<Double> rightTriggerAxis, Supplier<Double> leftTriggerAxis, 
            Supplier<Boolean> neutralTigger, Supplier<Boolean> speakerButton, Supplier<Boolean> ampButton,
            Supplier<Boolean> lowPassButton, Supplier<Boolean> longPassButton, Supplier<Boolean> climberButton) {
        this.shooterPositioner = shooterPositioner;
        this.shooter = shooter;
        this.indexer = indexer;
        this.climber = climber;
        this.rightTriggerAxis = rightTriggerAxis;
        this.leftTriggerAxis = leftTriggerAxis;

        addRequirements(climber);

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
        if (neutralTrigger.get() && climbStages < 3) {
            neutralState = true;
            speakerState = false;
            ampState = false;
            lowPassState = false;
            longPassState = false;
            climberState = false;
        }else if (speakerButton.get()) {
            neutralState = false;
            speakerState = true;
            ampState = false;
            lowPassState = false;
            longPassState = false;
            climberState = false;
        } else if(ampButton.get()) {
            neutralState = false;
            speakerState = false;
            ampState = true;
            lowPassState = false;
            longPassState = false;
            climberState = false;
        } else if(lowPassButton.get()) {
            neutralState = false;
            speakerState = false;
            ampState = false;
            lowPassState = true;
            longPassState = false;
            climberState = false;
        } else if(longPassButton.get()) {
            neutralState = false;
            speakerState = false;
            ampState = false;
            lowPassState = false;
            longPassState = true;
            climberState = false;
        } else if(climberButton.get()) {
            neutralState = false;
            speakerState = false;
            ampState = false;
            lowPassState = false;
            longPassState = false;
            climberState = true;
        }
    }

    public void setVelocity(double rpmVelocity) {
        shooter.setVelocity(rpmVelocity, rpmVelocity);
    }

    public boolean shootIsPressed() {
        return rightTriggerAxis.get() > 0.0;
    }

    public void indexerMove(double shootSetPoint, double angleSetPoint) {
        if (shooter.isAtSetPoint(shootSetPoint) && shooterPositioner.isAtSetPoint(angleSetPoint)) {
            // Move intake and indexer when the shooter it's at set point
            indexer.setPercentage(0.5);
        } else {
            indexer.setPercentage(0);
        }
    }

    public void shootAtVelocity(double velocity) {
        double shooterPercentage = rightTriggerAxis.get() - leftTriggerAxis.get();
        double rpms = VelocityConstants.Velocities.kAmpShooterVelocity * shooterPercentage;
        setVelocity(rpms);
    }

    public void moveShooterPositionerAtPosition(double position) {
        shooterPositioner.goToPosition(position);
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
            //  Shooter angle
            moveShooterPositionerAtNeutralPosition();
        }else if (speakerState) { 
            // Shooter
            shootAtSpeaker();

            moveShooterPositionerAtSpeaker();

            // Intake and indexer
            indexerMove(Constants.ShooterConstants.shootVelocity, shooterPositioner.getEstimatedShooterPositionerAngleWithRect());

        }else if (ampState) {
            // Shooter
            shootAtVelocity(VelocityConstants.Velocities.kAmpShooterVelocity);

            // Shooter angle
            moveShooterPositionerAtPosition(Constants.ShooterPositionerConstants.ampPosition);

        } /*else if (lowPassState) {
            // Shooter
            shootAtVelocity(VelocityConstants.Velocities.kLowPassShooterVelocity);

            // Shooter angle
            moveShooterPositionerAtPosition(Constants.ShooterPositionerConstants.lowPassPosition);

            // Intake and indexer
            indexerMove(VelocityConstants.Velocities.kLowPassShooterVelocity, Constants.ShooterPositionerConstants.lowPassPosition);
        } else if (longPassState) {
            // Shooter
            shootAtVelocity(VelocityConstants.Velocities.kLongPassShooterVelocity);

            // Shooter angle
            moveShooterPositionerAtPosition(Constants.ShooterPositionerConstants.longPassPosition);

            // Intake and indexer
            indexerMove(VelocityConstants.Velocities.kLongPassShooterVelocity, Constants.ShooterPositionerConstants.longPassPosition);
        } else if (climberState) {
            // Stage 1 raise de hooks
            if (!climber.isAtTopLimit() && climbStages == 0){
                climber.setVelocity(Constants.ClimberConstants.velocity, Constants.ClimberConstants.velocity);
                climbStages++;
            } 

            // Stage 2 move shooter positioner at trap position after the robot climbed
            if (climber.isAtBottomLimit()) {
                moveShooterPositionerAtPosition(Constants.ShooterPositionerConstants.trapPosition);
                if (shooterPositioner.isAtSetPoint(Constants.ShooterPositionerConstants.trapPosition)){
                    climbStages++;
                }
            }

            // Stage 3 shoot the note
            if (climbStages == 3) {
                // Shooter
                shootAtVelocity(VelocityConstants.Velocities.kAmpShooterVelocity);
                indexerMove(VelocityConstants.Velocities.kAmpShooterVelocity, Constants.ShooterPositionerConstants.ampPosition);
            }
        }*/
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}