package frc.robot.commands;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class RunAutoBalance extends CommandBase {
    private final Drivetrain drivetrain;

    public RunAutoBalance(Drivetrain drivetrain, boolean forwards) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(Math.abs(drivetrain.getPitch()) < 10) {
            double pitchRadians = Math.toRadians(drivetrain.getPitch());
            ChassisSpeeds speeds = new ChassisSpeeds(-Math.sin(pitchRadians), 0, 0);
            drivetrain.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
        } else {
            drivetrain.setModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(-3 * Math.PI / 4))
            });
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
