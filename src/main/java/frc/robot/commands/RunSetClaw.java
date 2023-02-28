package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Claw;

public class RunSetClaw extends CommandBase {
    private Claw claw;
    private boolean desiredState;

    public RunSetClaw(Claw claw, boolean forward) {
        this.claw = claw;
        this.desiredState = forward;
    }

    @Override
    public void initialize() {   
        claw.run(desiredState);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
