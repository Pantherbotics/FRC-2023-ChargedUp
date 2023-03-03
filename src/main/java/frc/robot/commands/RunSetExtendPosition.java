package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class RunSetExtendPosition extends CommandBase {
    private final Arm arm;

    private double position;

    public RunSetExtendPosition(Arm arm, double position) {
        this.arm = arm;
        this.position = position;

        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.setExtendPosition(position);
        System.out.println("extend to" + 30);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}