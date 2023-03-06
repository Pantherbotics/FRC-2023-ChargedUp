package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Claw;

public class RunToggleClaw extends CommandBase {
    private Claw claw;

    public RunToggleClaw(Claw claw) {
        this.claw = claw;
    }
    
    @Override
    public void initialize() {
        claw.toggle();
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
