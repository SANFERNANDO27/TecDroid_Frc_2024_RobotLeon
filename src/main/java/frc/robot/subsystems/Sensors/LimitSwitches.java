package frc.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.Constants;

public class LimitSwitches {
    private final DigitalInput shooterLimitSwitch = new DigitalInput(Constants.LimitSwitchConstants.shooterLimitSwitchId);
    private final DigitalInput shooterPositionerLimitSwitch = new DigitalInput(Constants.LimitSwitchConstants.shooterPositionerLimitSwitchId);

    public boolean getShooterLimitSwitch() {
        return !shooterLimitSwitch.get();
    }

    public boolean getShooterPositionerLimitSwitch() {
        return shooterPositionerLimitSwitch.get();
    }
}
