package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Extend;

public class RunExtendArm extends CommandBase {
    private Extend extend;
    private boolean reversed;

    public RunExtendArm(Extend extend, boolean reversed) {
        this.extend = extend;
        this.reversed = reversed;

        addRequirements(extend);
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        if(extend.getIsOpenLoop())
            extend.runOpenLoop(.5 * (reversed ? -1 : 1));
        else
            extend.runClosedLoop(2048.0 / 10.0 * (reversed ? -1 : 1));  
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(extend.getIsOpenLoop())
            extend.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
