package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Arm;

public class RunPivotArm extends CommandBase {
    private Arm arm;
    private boolean inverted;

    public RunPivotArm(Arm arm, boolean inverted) {
        this.arm = arm;
        this.inverted = inverted;
        
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        arm.pivot(inverted);
    }
  
    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.stopPivot();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
