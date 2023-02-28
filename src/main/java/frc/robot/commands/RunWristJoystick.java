package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Wrist;

public class RunWristJoystick extends CommandBase {
    private Wrist wrist;
    private Joystick joystick;

    public RunWristJoystick(Wrist wrist, Joystick joystick) {
        this.wrist = wrist;
        this.joystick = joystick;
        
        addRequirements(wrist);
    }

    @Override
    public void initialize() {}
  
    @Override
    public
     void execute() {
        double xLeftValue = joystick.getRawAxis(OIConstants.kSecondaryJoystickLeftXAxisID);
        double yRightValue = joystick.getRawAxis(OIConstants.kSecondaryJoystickRightYAxisID);

        double xSpeed = xLeftValue * xLeftValue * (xLeftValue < 0 ? -1 : 1);
        double ySpeed = yRightValue * yRightValue * (xLeftValue < 0 ? -1 : 1);

        if(Math.abs(xSpeed) > 1) xSpeed /= Math.abs(xSpeed); 
        if(Math.abs(ySpeed) > 1) ySpeed /= Math.abs(ySpeed); 

        xSpeed *= ArmConstants.kFlexMaxAngularSpeedDegreesPerSecond;
        ySpeed *= ArmConstants.kRotateMaxAngularSpeedDegreesPerSecond;

        if(wrist.flexOpenLoop) {
            wrist.flexOpenLoop(ySpeed);
            wrist.rotateOpenLoop(xSpeed);
        } else {
            wrist.flexClosedLoop(ySpeed);
            wrist.rotateClosedLoop(xSpeed);
        }
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
