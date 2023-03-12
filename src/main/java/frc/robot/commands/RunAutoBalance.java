package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    public void initialize() {}

    @Override
    public void execute() {
        if(Math.abs(drivetrain.getPitch()) >= 5) {
            double pitchRadians = Math.toRadians(drivetrain.getPitch());
            ChassisSpeeds speeds = new ChassisSpeeds(-Math.sin(pitchRadians), 0, 0);
            drivetrain.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
        } else {
            drivetrain.stop();
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
