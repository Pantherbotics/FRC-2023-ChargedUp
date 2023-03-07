package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AutoCommand;
import frc.robot.auto.AutoManager;
import frc.robot.commands.RunPivotArm;
import frc.robot.commands.RunSetExtendPosition;
import frc.robot.commands.RunSetPivotAngle;
import frc.robot.commands.RunSetWristPosition;
import frc.robot.commands.RunSwerveJoystick;
import frc.robot.commands.RunToggleClaw;
import frc.robot.commands.RunExtendArm;
import frc.robot.commands.RunWristJoystick;
import frc.robot.controllers.Controller;
import frc.robot.subsystems.arm.Extend;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.modes.DriveMode;
import frc.robot.subsystems.drive.modes.SpeedMode;
import frc.robot.subsystems.intake.Claw;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.vision.Limelight;

public class RobotContainer {
    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Limelight reflective = new Limelight("limelight-poggers");
    private final Limelight apriltag = new Limelight("limelight-pog");
    private final Extend extend = new Extend(); 
    private final Pivot pivot = new Pivot();
    private final Wrist wrist = new Wrist();
    private final Claw claw = new Claw();

    // Autos
    private final AutoManager autoManager = new AutoManager(drivetrain, reflective, apriltag, extend, pivot, wrist, claw);
    private final HashMap<String, Command> autoCommands = new HashMap<String, Command>();

    // Choosers
    private final SendableChooser<SpeedMode> speedModeChooser = new SendableChooser<SpeedMode>();
    private final SendableChooser<DriveMode> driveModeChooser = new SendableChooser<DriveMode>();
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    // Controllers
    private final Joystick primaryJoystick = new Joystick(OIConstants.kPrimaryJoystickID);
    private final Joystick secondaryJoystick = new Joystick(OIConstants.kSecondaryJoystickID);

    private final Controller primaryController = new Controller(OIConstants.kPrimaryJoystickID);
    private final Controller secondaryController = new Controller(OIConstants.kSecondaryJoystickID);

    // Primary controller buttons 
    private final JoystickButton primaryJoystickAButton = new JoystickButton(primaryJoystick, 1); // A Button
    private final JoystickButton primaryJoystickBButton = new JoystickButton(primaryJoystick, 2); // B Button
    private final JoystickButton primaryJoystickXButton = new JoystickButton(primaryJoystick, 3); // X Button
    private final JoystickButton primaryJoystickYButton = new JoystickButton(primaryJoystick, 4); // Y Button
    private final JoystickButton primaryJoystickLeftBumperButton = new JoystickButton(primaryJoystick, 5); // Left Bumper
    private final JoystickButton primaryJoystickRightBumperButton = new JoystickButton(primaryJoystick, 6); // Right Bumper
    private final JoystickButton primaryJoystickBackButton = new JoystickButton(primaryJoystick, 7); // Back Button
    private final JoystickButton primaryJoystickStartButton = new JoystickButton(primaryJoystick, 8); // Start Button
    private final JoystickButton primaryJoystickLeftJoystickButton = new JoystickButton(primaryJoystick, 9); // Left Stick Button
    private final JoystickButton primaryJoystickRightJoystickButton = new JoystickButton(primaryJoystick, 10); // Right Stick Button
                                                                                     
    private final POVButton primaryJoystickPOVNorth = new POVButton(primaryJoystick, 0); // North
    private final POVButton primaryJoystickPOVEast = new POVButton(primaryJoystick, 90); // East
    private final POVButton primaryJoystickPOVSouth = new POVButton(primaryJoystick, 180); // South
    private final POVButton primaryJoystickPOVWest = new POVButton(primaryJoystick, 270); // West

    // Secondary controller buttons 
    private final JoystickButton secondaryJoystickAButton = new JoystickButton(secondaryJoystick, 1); // Square Button
    private final JoystickButton secondaryJoystickBButton = new JoystickButton(secondaryJoystick, 2); // X Button
    private final JoystickButton secondaryJoystickXButton = new JoystickButton(secondaryJoystick, 3); // Circle Button
    private final JoystickButton secondaryJoystickYButton = new JoystickButton(secondaryJoystick, 4); // Triangle Button
    private final JoystickButton secondaryJoystickLeftBumperButton = new JoystickButton(secondaryJoystick, 5); // Right Bumper
    private final JoystickButton secondaryJoystickRightBumperButton = new JoystickButton(secondaryJoystick, 6); // Left Bumper
    private final JoystickButton secondaryJoystickBackButton = new JoystickButton(secondaryJoystick, 7); // Back Button
    private final JoystickButton secondaryJoystickStartButton = new JoystickButton(secondaryJoystick, 8); // Start Button
    private final JoystickButton secondaryJoystickLeftJoystickButton = new JoystickButton(secondaryJoystick, 9); // Left 
    private final JoystickButton secondaryJoystickRightJoystickButton = new JoystickButton(secondaryJoystick, 10); // Right
                                                                                                                
    private final POVButton secondaryJoystickPOVNorth = new POVButton(secondaryJoystick, 0); // North
    private final POVButton secondaryJoystickPOVEast = new POVButton(secondaryJoystick, 90); // East
    private final POVButton secondaryJoystickPOVSouth = new POVButton(secondaryJoystick, 180); // South
    private final POVButton secondaryJoystickPOVWest = new POVButton(secondaryJoystick, 270); // West

    public RobotContainer() {
        configAutoCommands();
        configButtonBindings();
        configChoosers();
    }

    private void configAutoCommands() {
        autoCommands.put(
            "None", 
            new PrintCommand("xdd")
        );
        autoCommands.put(
            "Taxi", 
            getPPCommand("SmallTaxi", true)
        );
        autoCommands.put(
            "Taxi Over Charge Station", 
            getPPCommand("LargeTaxi", true)
        );
        autoCommands.put(
            "Medium Goal", 
            new SequentialCommandGroup(
                new ParallelCommandGroup( //moves arm
                    new RunSetPivotAngle(pivot, 48),
                    new RunSetExtendPosition(extend, 0),
                    new RunSetWristPosition(wrist, -13000, 0)),
                new WaitCommand(1),
                new RunToggleClaw(claw),
                new WaitCommand(0.5),
                new ParallelCommandGroup( //zeros position
                    new RunSetPivotAngle(pivot, ArmConstants.kPivotZeroAngle),
                    new RunSetExtendPosition(extend, 0),
                    new RunSetWristPosition(wrist, 0, 0)
        )));
        autoCommands.put(
            "Medium Goal + Taxi", 
            new SequentialCommandGroup(
                new ParallelCommandGroup( 
                    new RunSetPivotAngle(pivot, 48),
                    new RunSetExtendPosition(extend, 0),
                    new RunSetWristPosition(wrist, -13000, 0)),
                new WaitCommand(1),
                new RunToggleClaw(claw),
                new WaitCommand(0.5),
                new ParallelCommandGroup( //zeros position
                    new RunSetPivotAngle(pivot, ArmConstants.kPivotZeroAngle),
                    new RunSetExtendPosition(extend, 0),
                    new RunSetWristPosition(wrist, 0, 0)),
                new WaitCommand(1),
                getPPCommand("GamePieceTaxi", true)
        ));
        autoCommands.put(
            "High Goal", 
            new SequentialCommandGroup(
                new ParallelCommandGroup( 
                    new RunSetPivotAngle(pivot, 48),
                    new RunSetExtendPosition(extend, 51000),
                    new RunSetWristPosition(wrist, -13000, 0)),
                new WaitCommand(1),
                new RunToggleClaw(claw),
                new WaitCommand(0.5),
                new ParallelCommandGroup( //zeros position
                    new RunSetPivotAngle(pivot, ArmConstants.kPivotZeroAngle),
                    new RunSetExtendPosition(extend, 0),
                    new RunSetWristPosition(wrist, 0, 0)
        )));
        autoCommands.put(
            "High Goal + Taxi", 
            new SequentialCommandGroup(
                new ParallelCommandGroup( 
                    new RunSetPivotAngle(pivot, 48),
                    new RunSetExtendPosition(extend, 51000),
                    new RunSetWristPosition(wrist, -13000, 0)),
                new WaitCommand(1),
                new RunToggleClaw(claw),
                new WaitCommand(0.5),
                new ParallelCommandGroup( //zeros position
                    new RunSetPivotAngle(pivot, ArmConstants.kPivotZeroAngle),
                    new RunSetExtendPosition(extend, 0),
                    new RunSetWristPosition(wrist, 0, 0)),
                new WaitCommand(1),
                getPPCommand("GamePieceTaxi", true)
        ));
    }

    private Command getPPCommand(String pathName, boolean firstPath) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(
            pathName, 
            new PathConstraints(
                AutoConstants.kMaxSpeedMetersPerSecond, 
                AutoConstants.kMaxAccelerationMetersPerSecondSquared
            ));
        return new SequentialCommandGroup(
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
    }

    private void configButtonBindings() {
        // drivetrain manual control
        drivetrain.setDefaultCommand(new RunSwerveJoystick(
            drivetrain,
            primaryJoystick,
            speedModeChooser::getSelected,
            driveModeChooser::getSelected
        ));
        primaryJoystickYButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

        // wrist manual control
        wrist.setDefaultCommand(new RunWristJoystick(wrist, secondaryJoystick));

        // pivot manual control
        secondaryJoystickXButton.whileTrue(new RunPivotArm(pivot, true));
        secondaryJoystickYButton.whileTrue(new RunPivotArm(pivot, false));

        // extension manual control 
        secondaryJoystickLeftBumperButton.whileTrue(new RunExtendArm(extend, true));
        secondaryJoystickRightBumperButton.whileTrue(new RunExtendArm(extend, false));

        // claw manual control
        secondaryJoystickAButton.toggleOnTrue(new RunToggleClaw(claw));

        // zero position
        secondaryJoystickBButton.whileTrue(new ParallelCommandGroup(
            new RunSetPivotAngle(pivot, ArmConstants.kPivotZeroAngle),
            new RunSetExtendPosition(extend, 0),
            new RunSetWristPosition(wrist, 0, 0)
        ));
        // high goal
        secondaryJoystickPOVNorth.whileTrue(new ParallelCommandGroup( 
            new RunSetPivotAngle(pivot, 48),
            new RunSetExtendPosition(extend, 51000),
            new RunSetWristPosition(wrist, -13000, 0)
        ));
        // medium goal
        secondaryJoystickPOVEast.whileTrue(new SequentialCommandGroup( 
            new RunSetPivotAngle(pivot, 48),
            new RunSetExtendPosition(extend, 0),
            new RunSetWristPosition(wrist, -13000, 0)
        ));
        // shelf
        secondaryJoystickPOVWest.whileTrue(new SequentialCommandGroup(
            new RunSetPivotAngle(pivot, 70),
            new RunSetExtendPosition(extend, 0),
            new RunSetWristPosition(wrist, -13000, 0)
        ));                                                                
        // picking off ground   
        secondaryJoystickPOVSouth.whileTrue(new SequentialCommandGroup(
            new RunSetPivotAngle(pivot, 5),
            new RunSetExtendPosition(extend, 0),
            new RunSetWristPosition(wrist, -15000, 0)
        ));
    }

    private void configChoosers() {
        // speed mode chooser
        for(SpeedMode speed : SpeedMode.values())
        {
            if(speed.equals(drivetrain.getSpeedMode()))
                speedModeChooser.setDefaultOption(speed.toString(), speed);
            else
                speedModeChooser.addOption(speed.toString(), speed);
        }
        SmartDashboard.putData(speedModeChooser);

        // drive mode chooser
        for(DriveMode mode : DriveMode.values())
        {
            if(mode.equals(drivetrain.getDriveMode()))
                driveModeChooser.setDefaultOption(mode.toString(), mode);
            else
                driveModeChooser.addOption(mode.toString(), mode);
        }
        SmartDashboard.putData(driveModeChooser);

        // auto chooser
        for(AutoCommand auto : autoManager.geAutoCommands())
        {
            if(auto.getName().equals("None"))
                autoChooser.setDefaultOption(auto.getName(), auto.getCommand());
            else
                autoChooser.addOption(auto.getName(), auto.getCommand());
        }
        SmartDashboard.putData(autoChooser);
    }

    public void updateSmartDashboard() {
        // drivetrain
        SmartDashboard.putNumber("Drivetrain Heading", drivetrain.getHeading());

        SmartDashboard.putNumber("Front Left [1] Speed", drivetrain.getFrontLeft().getDriveVelocity());
        SmartDashboard.putNumber("Front Left [1] Angle", drivetrain.getFrontLeft().getAngle());
        
        SmartDashboard.putNumber("Front Right [2] Speed", drivetrain.getFrontRight().getDriveVelocity());
        SmartDashboard.putNumber("Front Right [2] Angle", drivetrain.getFrontRight().getAngle());
        
        SmartDashboard.putNumber("Back Right [3] Speed", drivetrain.getBackRight().getDriveVelocity());
        SmartDashboard.putNumber("Back Right [3] Angle", drivetrain.getBackRight().getAngle());
        
        SmartDashboard.putNumber("Back Left [4] Speed", drivetrain.getBackLeft().getDriveVelocity());
        SmartDashboard.putNumber("Back Left [4] Angle", drivetrain.getBackLeft().getAngle());

        drivetrain.setDriveMode(driveModeChooser.getSelected());
        drivetrain.setSpeedMode(speedModeChooser.getSelected());

        // limelights

        // arm
        SmartDashboard.putNumber("Pivot Setpoint", pivot.getSetpoint());
        SmartDashboard.putNumber("Pivot Position", pivot.getAngle());

        SmartDashboard.putNumber("Extend Setpoint", extend.getSetpoint());
        SmartDashboard.putNumber("Extend Position", extend.getPosition());

        // wrist
        SmartDashboard.putNumber("Wrist Flex Setpoint", wrist.getFlexSetpoint());
        SmartDashboard.putNumber("Wrist Flex Position", wrist.getFlexAngle());

        SmartDashboard.putNumber("Wrist Rotate Setpoint", wrist.getRotateSetpoint());
        SmartDashboard.putNumber("Wrist Rotate Position", wrist.getRotateAngle());
         
        // claw
        SmartDashboard.putBoolean("Claw open?", claw.isOpen());
    }

    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }
}
