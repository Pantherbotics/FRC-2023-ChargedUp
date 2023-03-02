package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Claw extends SubsystemBase {
    private final DoubleSolenoid solenoid;
    private boolean isOpen;

    public Claw() {
        solenoid = new DoubleSolenoid(ArmConstants.kClawSolenoidPort, PneumaticsModuleType.CTREPCM, 0, 1);
        isOpen = false;
    }

    public void run(boolean state) {
        solenoid.set(state ? Value.kForward : Value.kReverse);
        isOpen = state;
    }

    public void toggle() {
        switch(solenoid.get()) {
            case kForward:
                solenoid.set(Value.kReverse);
                isOpen = true;
                break;
            case kReverse:
                solenoid.set(Value.kForward);
                isOpen = false;
                break;
            default:
                solenoid.set(Value.kOff);
                isOpen = false;
                break;
        }
    }

    public void stop() {
        solenoid.set(Value.kOff);
        isOpen = false;
    }

    public boolean isOpen() {
        return isOpen;
    }
}
