package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Wrist;

public class RunWrist extends CommandBase {
    private Wrist wrist;
    private Joystick joystick;

    public RunWrist(Wrist wrist, Joystick joystick) {
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

        double xSpeed = xLeftValue * 1;
        double ySpeed = yRightValue * 1;

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;

        xSpeed *= ArmConstants.kWristMaxAngularSpeedDegreesPerSecond;
        ySpeed *= ArmConstants.kWristMaxAngularSpeedDegreesPerSecond;

        if(wrist.openLoop) {
            wrist.flexOpenLoop(ySpeed);
            wrist.rotateOpenLoop(xSpeed);
        } else {
            wrist.flexClosedLoop(ySpeed);
            wrist.rotateClosedLoop(xSpeed);
        }
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // wrist.stopFlex();
        // wrist.stopRotate();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
