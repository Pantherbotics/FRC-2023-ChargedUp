package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class RunExtendArm extends CommandBase {
    private Arm arm;
    private boolean reversed;

    public RunExtendArm(Arm arm, boolean reversed) {
        this.arm = arm;
        this.reversed = reversed;

        addRequirements(arm);
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        if(arm.extendOpenLoop)
            arm.extendOpenLoop(.5 * (reversed ? -1 : 1));
        else
            arm.extendClosedLoop(2048.0 / 10.0 * (reversed ? -1 : 1));  
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(arm.extendOpenLoop)
            arm.stopExtension();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
