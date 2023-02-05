package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Arm;

public class RunTelescopicArm extends CommandBase {
    private Arm arm;
    private Joystick joystick;

    public RunTelescopicArm(Arm arm, Joystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        double xLeftValue = joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftXAxisID);
        double yLeftValue = joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID);
  
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.stopExtension();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
