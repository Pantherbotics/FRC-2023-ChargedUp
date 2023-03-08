package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.commands.RunPivotArmJoystick;
import frc.robot.commands.RunSetExtendPosition;
import frc.robot.commands.RunSetPivotAngle;
import frc.robot.commands.RunSetWristPosition;
import frc.robot.commands.RunDrivetrainJoystick;
import frc.robot.commands.RunToggleClaw;
import frc.robot.commands.RunExtendArmJoystick;
import frc.robot.commands.RunWristJoystick;
import frc.robot.oi.Controller;
import frc.robot.subsystems.arm.Extend;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.SwerveModule;
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

    // Event map for button binding and autos
    private final HashMap<String, Command> eventMap = new HashMap<String, Command>();

    // Auto stuff
    private final AutoManager autoManager = new AutoManager();
    // private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    //     drivetrain::getPose, 
    //     drivetrain::resetOdometry, 
    //     DriveConstants.kDriveKinematics, 
    //     new PIDConstants(
    //         AutoConstants.kPXController, 
    //         AutoConstants.kIXYController, 
    //         AutoConstants.kDXYController
    //     ),
    //     new PIDConstants(
    //         AutoConstants.kPThetaController, 
    //         AutoConstants.kIThetaController, 
    //         AutoConstants.kDThetaController
    //     ),
    //     drivetrain::setModuleStates, 
    //     eventMap,
    //     true, 
    //     drivetrain
    // );

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
        initEventMap();
        configAutos();
        configButtonBindings();
    }

    private void initEventMap() {
        eventMap.put(
            "Stow", 
            new ParallelCommandGroup(
                new RunSetPivotAngle(pivot, ArmConstants.kPivotZeroAngle),
                new RunSetExtendPosition(extend, 0),
                new RunSetWristPosition(wrist, 0, 0)
        ));
    }

    private void configAutos() {

    }

    private void configButtonBindings() {
        // drivetrain manual control
        drivetrain.setDefaultCommand(new RunDrivetrainJoystick(primaryJoystick, drivetrain));

        primaryJoystickXButton.onTrue(eventMap.get("Stow"));
        primaryJoystickYButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
        primaryJoystickAButton.onTrue(new InstantCommand(() -> drivetrain.setDriveMode(DriveMode.ROBOT_ORIENTED_SWERVE)));
        primaryJoystickBButton.onTrue(new InstantCommand(() -> drivetrain.setDriveMode(DriveMode.FIELD_ORIENTED_SWERVE)));

        primaryJoystickPOVNorth.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.SLOW)));
        primaryJoystickPOVEast.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.KINDA_SLOW)));
        primaryJoystickPOVSouth.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.NORMAL)));
        primaryJoystickPOVWest.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.FAST)));

        // wrist manual control
        wrist.setDefaultCommand(new RunWristJoystick(secondaryJoystick, wrist));

        // pivot manual control
        pivot.setDefaultCommand(new RunPivotArmJoystick(secondaryJoystick, pivot));
        
        // extension manual control 
        extend.setDefaultCommand(new RunExtendArmJoystick(secondaryJoystick, extend));

        // claw manual control
        secondaryJoystickAButton.toggleOnTrue(new RunToggleClaw(claw));

        // stow
        secondaryJoystickBButton.onTrue(new ParallelCommandGroup(
            new RunSetPivotAngle(pivot, ArmConstants.kPivotZeroAngle),
            new RunSetExtendPosition(extend, 0),
            new RunSetWristPosition(wrist, 0, 0)
        ));
        // high goal towards long end
        secondaryJoystickPOVNorth.onTrue(new ParallelCommandGroup(
            new RunSetPivotAngle(pivot, 48),
            new RunSetExtendPosition(extend, 51000),
            new RunSetWristPosition(wrist, 0, 0)
        ));
        // high goal towards short end
        secondaryJoystickYButton.onTrue(new ParallelCommandGroup(
            new RunSetPivotAngle(pivot, 150),
            new RunSetExtendPosition(extend, 40000),
            new RunSetWristPosition(wrist, -65, 0)
        ));
        // medium goal towards long end
        secondaryJoystickPOVEast.onTrue(new ParallelCommandGroup( 
            new RunSetPivotAngle(pivot, 48),
            new RunSetExtendPosition(extend, 0),
            new RunSetWristPosition(wrist, -65, 0)
        ));
        // medium goal towards short end
        secondaryJoystickXButton.onTrue(new ParallelCommandGroup(
            new RunSetPivotAngle(pivot, 150),
            new RunSetExtendPosition(extend, 0),
            new RunSetWristPosition(wrist, 0, 0)
        ));
        // shelf
        secondaryJoystickPOVWest.onTrue(new ParallelCommandGroup(
            new RunSetPivotAngle(pivot, 70),
            new RunSetExtendPosition(extend, 0),
            new RunSetWristPosition(wrist, -78, 0)
        ));                                                                
        // picking off ground   
        secondaryJoystickPOVSouth.onTrue(new ParallelCommandGroup(
            new RunSetPivotAngle(pivot, 5),
            new RunSetExtendPosition(extend, 0),
            new RunSetWristPosition(wrist, -45, 0)
        ));
    }

    public void updateSmartDashboard() {
        // speed mode chooser
        SendableChooser<SpeedMode> speedModeChooser = new SendableChooser<SpeedMode>();
        for(SpeedMode speedMode : SpeedMode.values()) {
            if(speedMode.equals(drivetrain.getSpeedMode()))
                speedModeChooser.setDefaultOption(speedMode.toString(), speedMode);
            else
                speedModeChooser.addOption(speedMode.toString(), speedMode);
        }
        SmartDashboard.putData(speedModeChooser);

        // drive mode chooser
        SendableChooser<DriveMode> driveModeChooser = new SendableChooser<DriveMode>();
        for(DriveMode driveMode : DriveMode.values()) {
            if(driveMode.equals(drivetrain.getDriveMode()))
                driveModeChooser.setDefaultOption(driveMode.toString(), driveMode);
            else
                driveModeChooser.addOption(driveMode.toString(), driveMode);
        }
        SmartDashboard.putData(driveModeChooser);

        // auto chooser
        SendableChooser<Command> autoChooser = new SendableChooser<Command>();
        autoManager.getAutos().forEach((name, auto) -> {
            if(auto.equals(autoManager.getSelectedAuto()))
                autoChooser.setDefaultOption(name, auto);
            else
                autoChooser.addOption(name, auto);
        });
        SmartDashboard.putData(autoChooser);

        // subsystems
        // drivetrain
        SmartDashboard.putNumber("Drivetrain Heading", drivetrain.getHeading());

        //drive modes
        SmartDashboard.putString("Drive Mode", drivetrain.getDriveMode().toString());
        SmartDashboard.putString("Speed Mode", drivetrain.getSpeedMode().toString());

        drivetrain.setDriveMode(driveModeChooser.getSelected());
        drivetrain.setSpeedMode(speedModeChooser.getSelected());

        // swerve modules

        for(SwerveModule module : drivetrain.getModules()) {
            String moduleName = "Swerve Module [" + module.getID() + "]";

            SmartDashboard.putNumber(moduleName + " Position", module.getDrivePosition());
            SmartDashboard.putNumber(moduleName + " Speed", module.getDriveVelocity());
            SmartDashboard.putNumber(moduleName + " Angle", module.getAngle());
            SmartDashboard.putNumber(moduleName + " Offset Angle", module.getOffsetAngle());
            SmartDashboard.putBoolean(moduleName + " Inverted", module.getInverted());

            module.setOffsetAngle(SmartDashboard.getNumber(moduleName + " Offset Degrees", module.getOffsetAngle()));
            module.setInverted(SmartDashboard.getBoolean(moduleName +" Inverted", module.getInverted()));
        }

        // arm
        //pivot
        SmartDashboard.putNumber("Pivot Setpoint", pivot.getSetpoint());
        SmartDashboard.putNumber("Pivot Position", pivot.getAngle());

        //extend
        SmartDashboard.putNumber("Extend Setpoint", extend.getSetpoint());
        SmartDashboard.putNumber("Extend Position", extend.getPosition());

        // wrist
        //flex
        SmartDashboard.putNumber("Flex Setpoint", wrist.getFlexSetpoint());
        SmartDashboard.putNumber("Flex Position", wrist.getFlexAngle());

        //rotate
        SmartDashboard.putNumber("Rotate Setpoint", wrist.getRotateSetpoint());
        SmartDashboard.putNumber("Rotate Position", wrist.getRotateAngle());
         
        // claw
        SmartDashboard.putBoolean("Claw open?", claw.isOpen());
    }

    public Command getAutoCommand() {
        return autoManager.getSelectedAuto();
    }
}
