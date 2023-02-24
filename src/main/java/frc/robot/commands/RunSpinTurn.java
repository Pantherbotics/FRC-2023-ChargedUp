package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Drivetrain;

public class RunSpinTurn extends CommandBase {
    private final Drivetrain drivetrain;
    private double angle;

    public RunSpinTurn(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        angle = (angle + 10) % 360;
        System.out.println(angle);
    }

    @Override
    public void execute() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        Arrays.fill(states, new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
        drivetrain.setModuleStates(states);
    }

    @Override
    public void end(boolean interrupted) {}
}