package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final DoubleSolenoid solenoidA;

    public Claw() {
        solenoidA = new DoubleSolenoid(9, PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void run(boolean state){
        solenoidA.set(state ? Value.kForward : Value.kReverse);
    }

    public void stop(){
        solenoidA.set(Value.kOff);
    }

    public void toggle(){
        switch (solenoidA.get()) {
            case kForward:
                solenoidA.set(Value.kReverse);
                break;
            case kReverse:
                solenoidA.set(Value.kForward);
                break;
            default:
                solenoidA.set(Value.kOff);
                break;
        }
    }
}