package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.intake.Wrist;

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

        double xSpeed = xLeftValue * ArmConstants.kRotateMaxAngularSpeedDegreesPerSecond;
        double ySpeed = yRightValue * ArmConstants.kFlexMaxAngularSpeedDegreesPerSecond;

        if(wrist.flexOpenLoop)
            wrist.flexOpenLoop(yRightValue);
        else
            wrist.flexClosedLoop(ySpeed);

        if(wrist.rotateOpenLoop)
            wrist.rotateOpenLoop(xLeftValue);
        else
            wrist.rotateClosedLoop(xSpeed);
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(wrist.flexOpenLoop)
            wrist.stopFlex();
        
        if(wrist.rotateOpenLoop)
            wrist.stopRotate();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
