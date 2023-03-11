package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AutoManager;
import frc.robot.commands.RunPivotArmJoystick;
import frc.robot.commands.RunSetClaw;
import frc.robot.commands.RunSetExtendPosition;
import frc.robot.commands.RunSetFlexAngle;
import frc.robot.commands.RunSetPivotAngle;
import frc.robot.commands.RunSetRotateAngle;
import frc.robot.commands.RunAutoBalance;
import frc.robot.commands.RunDrivetrainJoystick;
import frc.robot.commands.RunToggleClaw;
import frc.robot.commands.RunExtendArmJoystick;
import frc.robot.commands.RunWristJoystick;
import frc.robot.oi.OI;
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
    private final Limelight limelight = new Limelight();
    private final Extend extend = new Extend(); 
    private final Pivot pivot = new Pivot();
    private final Wrist wrist = new Wrist();
    private final Claw claw = new Claw();

    // Event map for button binding and autos
    private final HashMap<String, Command> eventMap = new HashMap<String, Command>();

    // Auto stuff
    private final String[] autoNames = {"Taxi", "High Goal", "High Goal and Taxi", "Two High Goal",  "Auto Balance", "High Goal and Auto Balance"};
    private final AutoManager autoManager = new AutoManager();
    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drivetrain::getPose, 
        drivetrain::resetOdometry, 
        DriveConstants.kDriveKinematics, 
        new PIDConstants(
            AutoConstants.kPXYController, 
            AutoConstants.kIXYController, 
            AutoConstants.kDXYController
        ),
        new PIDConstants(
            AutoConstants.kPThetaController, 
            AutoConstants.kIThetaController, 
            AutoConstants.kDThetaController
        ),
        drivetrain::setModuleStates, 
        eventMap,
        true, 
        drivetrain
    );

    //Choosers
    SendableChooser<SpeedMode> speedModeChooser = new SendableChooser<SpeedMode>();
    SendableChooser<DriveMode> driveModeChooser = new SendableChooser<DriveMode>();
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    // Operator Interface
    private OI oi;

    // Controllers
    private final Joystick primaryJoystick = new Joystick(OIConstants.kPrimaryJoystickID);
    private final Joystick secondaryJoystick = new Joystick(OIConstants.kSecondaryJoystickID);

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
        configChoosers();

        // default commands
        drivetrain.setDefaultCommand(new RunDrivetrainJoystick(primaryJoystick, drivetrain));
        wrist.setDefaultCommand(new RunWristJoystick(secondaryJoystick, wrist));
        pivot.setDefaultCommand(new RunPivotArmJoystick(secondaryJoystick, pivot));
        extend.setDefaultCommand(new RunExtendArmJoystick(secondaryJoystick, extend));

        configButtonBindings();
    }

    private void initEventMap() {
        eventMap.put(
            "Wait", 
            new WaitCommand(1));

        eventMap.put(
            "Balance From Front", // coming onto the charge station from the side further away from the driver stations
            new RunAutoBalance(drivetrain, true) 
        );
        eventMap.put(
            "Balance From Behind",
            new RunAutoBalance(drivetrain, false));

        //claw
        eventMap.put(
            "Open Claw", 
            new RunSetClaw(claw, true)
        );
        eventMap.put(
            "Close Claw", 
            new RunSetClaw(claw, false)
        );
        eventMap.put(
            "Toggle Claw", 
            new RunToggleClaw(claw)
        );

        //arm positions
        eventMap.put(
            "Stow", 
            new ParallelCommandGroup(
                new RunSetPivotAngle(pivot, ArmConstants.kPivotZeroAngle),
                new RunSetExtendPosition(extend, 0),
                new RunSetFlexAngle(wrist, -38)
        ));
        eventMap.put(
            "High Goal Towards Long End", 
            new ParallelCommandGroup(
                new RunSetPivotAngle(pivot, 68),
                new RunSetExtendPosition(extend, 49000),
                new RunSetFlexAngle(wrist, -47)
        ));
        eventMap.put(
            "High Goal Towards Short End",
            new ParallelCommandGroup(
                new RunSetPivotAngle(pivot, 183),
                new RunSetExtendPosition(extend, 26700),
                new RunSetFlexAngle(wrist, -61)
        ));
        eventMap.put(
            "Medium Goal Towards Long End",
            new ParallelCommandGroup( 
                new RunSetPivotAngle(pivot, 66),
                new RunSetExtendPosition(extend, 0),
                new RunSetFlexAngle(wrist, -46)
        ));
        eventMap.put(
            "Medium Goal Towards Short End",
            new ParallelCommandGroup( 
                new RunSetPivotAngle(pivot, 189),
                new RunSetExtendPosition(extend, 0),
                new RunSetFlexAngle(wrist, -62)
        )); 
        eventMap.put(
            "Pickup From Shelf",
            new ParallelCommandGroup(
                new RunSetPivotAngle(pivot, 85),
                new RunSetExtendPosition(extend, 0),
                new RunSetFlexAngle(wrist, -71)
        ));                                                                
        eventMap.put(
            "Pickup Off Ground",
            new ParallelCommandGroup(   
                new RunSetPivotAngle(pivot, 24),
                new RunSetExtendPosition(extend, 0),
                new RunSetFlexAngle(wrist, -47)
        ));
    }

    private void configAutos() {
        autoManager.addDefaultOption(
            "None",     
            new PrintCommand("No Auto"));
        for(String name : autoNames) {
            List<PathPlannerTrajectory> traj = PathPlanner.loadPathGroup(
                name, 
                AutoConstants.kMaxSpeedMetersPerSecond, 
                AutoConstants.kMaxAccelerationMetersPerSecondSquared
            );
            Command auto = autoBuilder.fullAuto(traj);
            autoManager.addOption(name, auto);
        }
    }

    private void configChoosers() {
        // speed mode chooser
        for(SpeedMode speedMode : SpeedMode.values()) {
            if(speedMode.equals(drivetrain.getSpeedMode()))
                speedModeChooser.setDefaultOption(speedMode.toString(), speedMode);
            else
                speedModeChooser.addOption(speedMode.toString(), speedMode);
        }
        SmartDashboard.putData(speedModeChooser);

        // drive mode chooser
        for(DriveMode driveMode : DriveMode.values()) {
            if(driveMode.equals(drivetrain.getDriveMode()))
                driveModeChooser.setDefaultOption(driveMode.toString(), driveMode);
            else
                driveModeChooser.addOption(driveMode.toString(), driveMode);
        }
        SmartDashboard.putData(driveModeChooser);

        // auto chooser
        autoManager.getAutos().forEach((name, auto) -> {
            if(auto.equals(autoManager.getSelectedAuto()))
                autoChooser.setDefaultOption(name, auto);
            else
                autoChooser.addOption(name, auto);
        });
        SmartDashboard.putData(autoChooser);
    }

    private void configButtonBindings() {
        // drivetrain manual control

        primaryJoystickXButton.onTrue(eventMap.get("Stow"));
        primaryJoystickYButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
        primaryJoystickAButton.onTrue(new InstantCommand(() -> drivetrain.setDriveMode(DriveMode.ROBOT_ORIENTED_SWERVE)));
        primaryJoystickBButton.onTrue(new InstantCommand(() -> drivetrain.setDriveMode(DriveMode.FIELD_ORIENTED_SWERVE)));

        primaryJoystickPOVNorth.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.SLOW)));
        primaryJoystickPOVEast.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.KINDA_SLOW)));
        primaryJoystickPOVSouth.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.NORMAL)));
        primaryJoystickPOVWest.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.FAST)));

        // claw manual control
        secondaryJoystickAButton.onTrue(eventMap.get("Toggle Claw"));

        secondaryJoystickBButton.onTrue(eventMap.get("Stow"));
        secondaryJoystickPOVNorth.onTrue(eventMap.get("High Goal Towards Long End"));
        secondaryJoystickYButton.onTrue(eventMap.get("High Goal Towards Short End"));
        secondaryJoystickPOVEast.onTrue(eventMap.get("Medium Goal Towards Long End"));
        secondaryJoystickXButton.onTrue(eventMap.get("Medium Goal Towards Short End")); 
        secondaryJoystickPOVWest.onTrue(eventMap.get("Pickup From Shelf"));                                                                
        secondaryJoystickPOVSouth.onTrue(eventMap.get("Pickup Off Ground"));
    }

    public void updateSmartDashboard() {
        autoManager.setSelectedAuto(autoChooser.getSelected());

        // drivetrain
        SmartDashboard.putNumber("Drivetrain Yaw", drivetrain.getYaw());
        SmartDashboard.putNumber("Drivetrain Pitch", drivetrain.getPitch());

        // drive modes
        SmartDashboard.putString("Drive Mode", drivetrain.getDriveMode().toString());
        SmartDashboard.putString("Speed Mode", drivetrain.getSpeedMode().toString());

        if(!DriverStation.isTeleopEnabled()) {
            drivetrain.setDriveMode(driveModeChooser.getSelected());
            drivetrain.setSpeedMode(speedModeChooser.getSelected());
        }

        // swerve modules
        for(SwerveModule module : drivetrain.getModules()) {
            String moduleName = "Swerve Module [" + module.getID() + "]";

            SmartDashboard.putNumber(moduleName + " Speed", module.getDriveVelocity());
            SmartDashboard.putNumber(moduleName + " Angle", module.getAngle());
            SmartDashboard.putNumber(moduleName + " Offset", module.getOffsetAngle());
            SmartDashboard.putBoolean(moduleName + " Inverted", module.getInverted());

            module.setOffsetAngle(SmartDashboard.getNumber(moduleName + " Offset", module.getOffsetAngle()));
            module.setInverted(SmartDashboard.getBoolean(moduleName +" Inverted", module.getInverted()));
        }

        // pivot
        SmartDashboard.putNumber("Pivot Setpoint", pivot.getSetpoint());
        SmartDashboard.putNumber("Pivot Position", pivot.getAngle());

        // extend
        SmartDashboard.putNumber("Extend Setpoint", extend.getSetpoint());
        SmartDashboard.putNumber("Extend Position", extend.getPosition());

        // flex
        SmartDashboard.putNumber("Flex Setpoint", wrist.getFlexSetpoint());
        SmartDashboard.putNumber("Flex Position", wrist.getFlexAngle());

        // rotate
        SmartDashboard.putNumber("Rotate Setpoint", wrist.getRotateSetpoint());
        SmartDashboard.putNumber("Rotate Position", wrist.getRotateAngle());
         
        // claw
        SmartDashboard.putString("Claw State", claw.isOpen() ? "Open" : "Close");
    }

    public Command getAutoCommand() {
        return autoManager.getSelectedAuto();
    }
}
