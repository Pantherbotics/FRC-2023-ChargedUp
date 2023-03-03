package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Claw extends SubsystemBase {
    private final Value OPEN_STATE = Value.kForward;

    private final DoubleSolenoid solenoid;
    private boolean open;

    public Claw() {
        solenoid = new DoubleSolenoid(ArmConstants.kClawSolenoidPort, PneumaticsModuleType.CTREPCM, 0, 1);
        open = solenoid.get().equals(OPEN_STATE);
    }

    public void run(boolean desiredState) {
        open = desiredState;
    }

    public void toggle() {
        open = !open;
    }

    public void stop() {
        open = false;
    }

    public boolean isOpen() {
        return open;
    }

    @Override
    public void periodic() {
        Value desiredState = open ? Value.kReverse : Value.kForward;
        if(!solenoid.get().equals(desiredState))
            solenoid.set(desiredState);
    }
}
