package frc.robot.auto;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.RunSetExtendPosition;
import frc.robot.commands.RunSetPivotAngle;
import frc.robot.commands.RunSetWristPosition;
import frc.robot.commands.RunToggleClaw;
import frc.robot.subsystems.arm.Extend;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.intake.Claw;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.vision.Limelight;

public class AutoPaths {    
    //subsystems
    private final Drivetrain drivetrain;
    private final Limelight reflective;
    private final Limelight apriltag;
    private final Extend extend;
    private final Pivot pivot; 
    private final Wrist wrist;
    private final Claw claw;

    private HashMap<String, Command> paths = new HashMap<String, Command>();

    public AutoPaths(Drivetrain drivetrain, Limelight reflective, Limelight apriltag, Extend extend, Pivot pivot, Wrist wrist, Claw claw) {
        this.drivetrain = drivetrain;
        this.reflective = reflective;
        this.apriltag = apriltag;
        this.extend = extend;
        this.pivot = pivot;
        this.wrist = wrist;
        this.claw = claw;

        createPaths();
    }

    private void createPaths() {
        paths.put(
            "Taxi", 
            getCommandFromTrajectory("Taxi", true)
        );

        paths.put(
            "Taxi Over Charge Station", 
            getCommandFromTrajectory("TaxiOverRamp", true)
        );

        paths.put(
            "1 Medium Cone + Taxi", 
            new SequentialCommandGroup(
                new ParallelCommandGroup( 
                    new RunSetPivotAngle(pivot, 48),
                    new RunSetExtendPosition(extend, 0),
                    new RunSetWristPosition(wrist, -13000, 0)),
                new WaitCommand(0),
                new RunToggleClaw(claw),
                getCommandFromTrajectory("MediumConeTaxi", true)
        ));

        //TODO: add more trajectories w/ pp
    }

    private Command getCommandFromTrajectory(String pathName, boolean firstPath) {
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

    public HashMap<String, Command> getPaths() {
        return paths;
    }
}
