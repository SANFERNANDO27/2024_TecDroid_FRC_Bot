package frc.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.Constants;

public class LimitSwitches {
    private final DigitalInput leftLimitSwitch = new DigitalInput(Constants.LimitSwitchConstants.leftLimitSwitchId);
    private final DigitalInput rightLimitSwitch = new DigitalInput(Constants.LimitSwitchConstants.rightLimitSwitchId);

    public boolean getLeftLimitSwitch() {
        return leftLimitSwitch.get();
    }

    public boolean getRightLimitSwitch() {
        return rightLimitSwitch.get();
    }

    public boolean getLimitSwitchesRead() {
        return getLeftLimitSwitch() || getRightLimitSwitch();
    }
}
