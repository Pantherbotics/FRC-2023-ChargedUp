package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Claw;

public class RunClaw extends CommandBase {
    private final Claw claw;
    private final Joystick joystick;

    public RunClaw(Claw claw, Joystick joystick) {
        this.claw = claw;
        this.joystick = joystick;
        addRequirements(claw);
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        double xLeftValue = joystick.getRawAxis(OIConstants.kSecondaryJoystickLeftXAxisID);
        double yLeftValue = joystick.getRawAxis(OIConstants.kSecondaryJoystickLeftYAxisID);
  
        if(claw.getDoPID()) {
            claw.flexPID(yLeftValue);
            claw.rotatePID(xLeftValue);
        } else {
            claw.rotateOpenLoop(xLeftValue);
            claw.flexOpenLoop(yLeftValue);
        }
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        claw.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
  }
