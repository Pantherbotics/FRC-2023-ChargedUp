package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Wrist;

public class RunSetFlexAngle extends CommandBase {
    private final Wrist wrist;
    private final double flexAngle;
    
    public RunSetFlexAngle(Wrist wrist, double flexAngle) {
        this.wrist = wrist;
        this.flexAngle = flexAngle;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        wrist.setFlexAngle(flexAngle);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
