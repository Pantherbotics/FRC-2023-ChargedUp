package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Pivot;

public class RunPivotArm extends CommandBase {
    private Pivot pivot;
    private boolean reversed;
    
    public RunPivotArm(Pivot pivot, boolean reversed) {
        this.pivot = pivot;
        this.reversed = reversed;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(pivot.getIsOpenLoop())
            pivot.runOpenLoop(.3 * (reversed ? -1 : 1)); 
        else
            pivot.runClosedLoop(.45 * (reversed ? -1 : 1)); 
    }

    @Override
    public void end(boolean interrupted) {
        if(pivot.getIsOpenLoop())
            pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
