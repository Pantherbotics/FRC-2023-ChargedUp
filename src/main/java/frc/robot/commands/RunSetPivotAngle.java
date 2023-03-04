package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class RunSetPivotAngle extends CommandBase {
    private final Arm arm;
    private double angle;

    public RunSetPivotAngle(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;

        addRequirements(arm);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.setPivotAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
