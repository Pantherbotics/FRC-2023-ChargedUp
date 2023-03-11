package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Claw extends SubsystemBase {
    private final Value OPEN_STATE = Value.kForward;
    private final Value CLOSE_STATE = Value.kReverse;
    private final Value OFF_STATE = Value.kOff;

    private final DoubleSolenoid solenoid;

    public Claw() {
        solenoid = new DoubleSolenoid(ArmConstants.kClawSolenoidPort, PneumaticsModuleType.CTREPCM, 1, 0);
        close();
    }

    /**
     * @param open True if open, false if close
     */
    public void set(boolean open) {
        if(open)
            open();
        else
            close();
    }

    /**
     * Toggles the claw state, if open close and vice versa
     */
    public void toggle() {
        set(!isOpen());
    }

    /**
     * Opens the claw
     */
    public void open() {
        solenoid.set(OPEN_STATE);
    }

    /**
     * Closes the claw
     */
    public void close() {
        solenoid.set(CLOSE_STATE);
    }

    /**
     * "Stops" the claw, turns the solenoid off
     */
    public void stop() {
        solenoid.set(OFF_STATE);
    }

    public boolean isOpen() {
        return solenoid.get().equals(OPEN_STATE);
    }
}
