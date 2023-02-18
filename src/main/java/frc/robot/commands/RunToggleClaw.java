package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Claw;

public class RunToggleClaw extends CommandBase{
    private Claw claw;

    public RunToggleClaw(Claw _claw){
        this.claw = _claw;
    }
    
    @Override
    public void initialize() {   
        claw.toggle();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
