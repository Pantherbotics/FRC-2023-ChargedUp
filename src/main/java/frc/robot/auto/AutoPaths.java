package frc.robot.auto;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.RunSetExtendPosition;
import frc.robot.commands.RunSetPivotAngle;
import frc.robot.commands.RunSetWristPosition;
import frc.robot.commands.RunToggleClaw;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Claw;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.vision.Limelight;

public class AutoPaths {
    //subsystems
    private final Drivetrain drivetrain;
    private final Limelight reflective;
    private final Limelight apriltag;
    private final Arm arm; 
    private final Wrist wrist;
    private final Claw claw;

    private HashMap<String, Command> paths = new HashMap<String, Command>();

    private HashMap<String, Command> eventMap = new HashMap<String, Command>();

    public AutoPaths(
        Drivetrain drivetrain, 
        Limelight reflective, 
        Limelight apriltag, 
        Arm arm, 
        Wrist wrist, 
        Claw claw) {
        
        this.drivetrain = drivetrain;
        this.reflective = reflective;
        this.apriltag = apriltag;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;

        createPaths();
    }

    private void createPaths() {
        paths.put(
            "None", 
            null
        );

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
            new SequentialCommandGroup(new SequentialCommandGroup( 
                new RunSetPivotAngle(arm, 48),
                new RunSetExtendPosition(arm, 0),
                new RunSetWristPosition(wrist, -13000, 0)
            )))
            .andThen(new RunToggleClaw(claw)
            .andThen(getCommandFromTrajectory("MediumConeTaxi", true)
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
