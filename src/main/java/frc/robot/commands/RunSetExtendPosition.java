package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Extend;

public class RunSetExtendPosition extends CommandBase {
    private final Extend extend;

    private double position;

    public RunSetExtendPosition(Extend extend, double position) {
        this.extend = extend;
        this.position = position;

        addRequirements(extend);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        extend.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}