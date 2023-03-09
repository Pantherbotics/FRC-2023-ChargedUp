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
        double leftAxis = joystick.getRawAxis(OIConstants.kSecondaryJoystickLeftTriggerID);
        double rightAxis = joystick.getRawAxis(OIConstants.kSecondaryJoystickRightTriggerID);

        double speed = leftAxis - rightAxis;

        if(pivot.getIsOpenLoop())
            pivot.runOpenLoop(speed); 
        else
            pivot.runClosedLoop(speed);
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
