package frc.robot.auto;
import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.arm.Extend;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.Claw;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.vision.Limelight;

public class AutoManager {    
    //subsystems
    private final Drivetrain drivetrain;
    private final Limelight reflective;
    private final Limelight apriltag;
    private final Extend extend;
    private final Pivot pivot; 
    private final Wrist wrist;
    private final Claw claw;

    private final ArrayList<AutoCommand> autoCommands = new ArrayList<AutoCommand>();

    public AutoManager(Drivetrain drivetrain, Limelight reflective, Limelight apriltag, Extend extend, Pivot pivot, Wrist wrist, Claw claw) {
        this.drivetrain = drivetrain;
        this.reflective = reflective;
        this.apriltag = apriltag;
        this.extend = extend;
        this.pivot = pivot;
        this.wrist = wrist;
        this.claw = claw;

        createCommands();
    }

    private void createCommands() {
        autoCommands.add(new AutoCommand("Short Taxi"));
        autoCommands.add(new AutoCommand("Long Taxi"));
        autoCommands.add(new AutoCommand("Medium Goal"));
        autoCommands.add(new AutoCommand("Medium Goal and Short Taxi"));
        autoCommands.add(new AutoCommand("Medium Goal and Long Taxi"));
        autoCommands.add(new AutoCommand("High Goal"));
        autoCommands.add(new AutoCommand("High Goal and Short Taxi"));
        autoCommands.add(new AutoCommand("High Goal and Long Taxi"));
    }

    private Command getCommandFromPP(String pathName, boolean firstPath) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(
            pathName, 
            new PathConstraints(
                AutoConstants.kMaxSpeedMetersPerSecond, 
                AutoConstants.kMaxAccelerationMetersPerSecondSquared
            ));
        SequentialCommandGroup command = new SequentialCommandGroup(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                if(firstPath)
                    drivetrain.resetOdometry(traj.getInitialState().holonomicRotation, traj.getInitialHolonomicPose());
            }),
            new PPSwerveControllerCommand(
                traj, 
                drivetrain::getPose,
                DriveConstants.kDriveKinematics, 
                new PIDController(AutoConstants.kPXController, 0, 0), 
                new PIDController(AutoConstants.kPYController, 0, 0), 
                new PIDController(AutoConstants.kPThetaController, 0, 0),
                drivetrain::setModuleStates,
                true, 
                drivetrain
        ));
        return command;
    }

    public ArrayList<AutoCommand> geAutoCommands() {
        return autoCommands;
    }
}
