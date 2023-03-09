package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Wrist;

public class RunSetRotateAngle extends CommandBase {
    private final Wrist wrist;
    private final double rotateAngle;
    
    public RunSetRotateAngle(Wrist wrist, double rotateAngle) {
        this.wrist = wrist;
        this.rotateAngle = rotateAngle;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        wrist.setRotateAngle(rotateAngle);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
