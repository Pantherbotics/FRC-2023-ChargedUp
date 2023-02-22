package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Wrist;

public class RunWrist extends CommandBase {
    private Wrist wrist;
    private Joystick joystick;

    public static final double kMaxAngularSpeedDegreesPerSecond = 90.0; 

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
        double xLeftValue = joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftXAxisID);
        double yRightValue = joystick.getRawAxis(OIConstants.kPrimaryJoystickRightYAxisID);

        double xSpeed = xLeftValue * 0.65;
        double ySpeed = yRightValue * 0.65;

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;

        wrist.setFlexAngle(ySpeed * kMaxAngularSpeedDegreesPerSecond);
        wrist.setRotateAngle(xSpeed * kMaxAngularSpeedDegreesPerSecond);
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wrist.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
