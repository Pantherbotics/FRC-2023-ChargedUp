package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Pivot;

public class RunPivotArmJoystick extends CommandBase {
    private final Joystick joystick;
    private final Pivot pivot;

    public RunPivotArmJoystick(Joystick joystick, Pivot pivot) {
        this.joystick = joystick;
        this.pivot = pivot;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {}
  
    @Override
    public
     void execute() {
        double xLeftValue = joystick.getRawAxis(OIConstants.kSecondaryJoystickLeftTriggerID);
        double yRightValue = -joystick.getRawAxis(OIConstants.kSecondaryJoystickRightTriggerID);

        if(pivot.getIsOpenLoop())
        {
            pivot.runOpenLoop(0.3 * xLeftValue); 
            pivot.runOpenLoop(0.3 * yRightValue);
        } else {
            pivot.runClosedLoop(0.6 * xLeftValue);
            pivot.runClosedLoop(0.6 * yRightValue);
        }
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(pivot.getIsOpenLoop())
            pivot.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
