package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class RunTelescopicArm extends CommandBase {
    private Arm arm;
    private boolean inverted;

    public RunTelescopicArm(Arm arm, boolean inverted) {
        this.arm = arm;
        this.inverted = inverted;

        addRequirements(arm);
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        arm.telescope(inverted);
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.stopTelescope();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
