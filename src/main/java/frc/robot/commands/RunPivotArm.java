package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class RunPivotArm extends CommandBase {
    private Arm arm;
    private boolean reversed;
    private boolean openLoop;
    
    public RunPivotArm(Arm arm, boolean reversed) {
        this.arm = arm;
        this.reversed = reversed;
        openLoop = arm.pivotOpenLoop;

        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(openLoop)
            arm.pivotOpenLoop(.1 * (reversed ? -1 : 1)); 
        else
            arm.pivotClosedLoop(.44 * (reversed ? -1 : 1)); 
    }

    @Override
    public void end(boolean interrupted) {
        if(openLoop)
            arm.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
