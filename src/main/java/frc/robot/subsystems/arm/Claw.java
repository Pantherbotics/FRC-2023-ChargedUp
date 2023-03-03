package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Claw extends SubsystemBase {
    private final Value OPEN_STATE = Value.kForward;
    private final Value CLOSE_STATE = Value.kReverse;

    private final DoubleSolenoid solenoid;
    private boolean open;

    public Claw() {
        solenoid = new DoubleSolenoid(ArmConstants.kClawSolenoidPort, PneumaticsModuleType.CTREPCM, 0, 1);
        open = solenoid.get().equals(OPEN_STATE);
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
        open = false;
    }

    public boolean isOpen() {
        return solenoid.get().equals(OPEN_STATE);
    }
}
