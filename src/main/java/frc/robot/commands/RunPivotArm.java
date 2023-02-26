package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class RunPivotArm extends CommandBase {
    private Arm arm;
    private boolean reversed;
    
    public RunPivotArm(Arm arm, boolean reversed) {
        this.arm = arm;
        this.reversed = reversed;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        if(arm.pivotOpenLoop)
            arm.pivotOpenLoop(.1 * (reversed ? -1 : 1)); 
        else
            arm.pivotClosedLoop(.51 * (reversed ? -1 : 1));
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopPivot();
    }
}
