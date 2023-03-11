package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Claw;

public class RunSetClaw extends CommandBase {
    private Claw claw;
    private boolean desiredState;

    public RunSetClaw(Claw claw, boolean open) {
        this.claw = claw;
        this.desiredState = open;
    }

    @Override
    public void initialize() {   
        claw.set(desiredState);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
