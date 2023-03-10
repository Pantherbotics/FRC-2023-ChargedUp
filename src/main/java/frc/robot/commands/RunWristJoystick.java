package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.intake.Wrist;

public class RunWristJoystick extends CommandBase {
    private Joystick joystick;
    private Wrist wrist;

    public RunWristJoystick(Joystick joystick, Wrist wrist) {
        this.joystick = joystick;
        this.wrist = wrist;
        
        addRequirements(wrist);
    }

    @Override
    public void initialize() {}
  
    @Override
    public
     void execute() {
        double rightXAxis = joystick.getRawAxis(OIConstants.kSecondaryJoystickRightXAxisID);
        double rightYAxis = -joystick.getRawAxis(OIConstants.kSecondaryJoystickRightYAxisID);

        if(wrist.getIsFlexOpenLoop())
            wrist.flexOpenLoop(rightYAxis);
        else
            wrist.flexClosedLoop(rightYAxis);

        if(wrist.getIsRotateOpenLoop())
            wrist.rotateOpenLoop(rightXAxis);
        else
            wrist.rotateClosedLoop(rightXAxis);
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(wrist.getIsFlexOpenLoop())
            wrist.stopFlex();
        
        if(wrist.getIsRotateOpenLoop())
            wrist.stopRotate();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
