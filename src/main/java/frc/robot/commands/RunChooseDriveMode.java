package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;

public class RunChooseDriveMode extends CommandBase {
    private final Drivetrain drivetrain;
    private final DriveMode driveMode;
    
    public RunChooseDriveMode(Drivetrain drivetrain, DriveMode driveMode) {
        this.drivetrain = drivetrain;
        this.driveMode = driveMode;
    }

    @Override
    public void initialize() {
        drivetrain.setDriveMode(driveMode);
        System.out.println("Switching the drive mode");
    }

    @Override
    public void execute() {
        System.out.println("Switching the drive mode");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
