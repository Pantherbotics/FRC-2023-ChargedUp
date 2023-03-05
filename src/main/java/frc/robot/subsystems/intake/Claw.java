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
        solenoid = new DoubleSolenoid(ArmConstants.kClawSolenoidPort, PneumaticsModuleType.CTREPCM, 0, 1);
    }

    /**
     * @param open True if open, false if close
     * 
     */
    public void set(boolean open) {
        if(open)
            open();
        else
            close();
    }

    public void toggle() {
        set(!isOpen());
    }

    public void open() {
        solenoid.set(OPEN_STATE);
    }

    public void close() {
        solenoid.set(CLOSE_STATE);
    }


    public void stop() {
        solenoid.set(OFF_STATE);
    }

    public boolean isOpen() {
        return solenoid.get().equals(OPEN_STATE);
    }
}
