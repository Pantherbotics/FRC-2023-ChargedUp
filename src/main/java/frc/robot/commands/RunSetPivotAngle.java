package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Pivot;

public class RunSetPivotAngle extends CommandBase {
    private final Pivot pivot;
    private double angle;

    public RunSetPivotAngle(Pivot pivot, double angle) {
        this.pivot = pivot;
        this.angle = angle;

        addRequirements(pivot);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        pivot.setAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
