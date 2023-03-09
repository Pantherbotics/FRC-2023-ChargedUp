package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Extend;

public class RunExtendArmJoystick extends CommandBase {
    private final Joystick joystick;
    private final Extend extend;

    public RunExtendArmJoystick(Joystick joystick, Extend extend) {
        this.joystick = joystick;
        this.extend = extend;

        addRequirements(extend);
    }

    @Override
    public void initialize() {}
  
    @Override
    public
     void execute() {
        double leftYAxis = -joystick.getRawAxis(OIConstants.kSecondaryJoystickLeftYAxisID);

        if(extend.getIsOpenLoop())
            extend.runOpenLoop(0.5 * leftYAxis);
        else
            extend.runClosedLoop(500 * leftYAxis); 
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
