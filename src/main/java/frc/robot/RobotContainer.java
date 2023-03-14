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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private final String[] autos = {
        "High Goal and Taxi", "High Goal", "Taxi", "Two High Goal Top Side", "Two High Goal", "Cone and Cube High Goal"
    };
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

        //gross
        SmartDashboard.putNumber("Swerve Module [1] Offset", drivetrain.getModules()[0].getOffsetAngle());
        SmartDashboard.putNumber("Swerve Module [2] Offset", drivetrain.getModules()[1].getOffsetAngle());
        SmartDashboard.putNumber("Swerve Module [3] Offset", drivetrain.getModules()[2].getOffsetAngle());
        SmartDashboard.putNumber("Swerve Module [4] Offset", drivetrain.getModules()[3].getOffsetAngle());

        SmartDashboard.putBoolean("Swerve Module [1] Inverted", drivetrain.getModules()[0].getInverted());
        SmartDashboard.putBoolean("Swerve Module [2] Inverted", drivetrain.getModules()[1].getInverted());
        SmartDashboard.putBoolean("Swerve Module [3] Inverted", drivetrain.getModules()[2].getInverted());
        SmartDashboard.putBoolean("Swerve Module [4] Inverted", drivetrain.getModules()[3].getInverted());

        SmartDashboard.putNumber("Pivot Offset", pivot.getOffsetAngle());
    }

    private void initEventMap() {
        eventMap.put(
            "Wait", 
            new WaitCommand(1));

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
                new RunSetFlexAngle(wrist, -18)
        ));
        eventMap.put(
            "High Goal Long End", 
            new ParallelCommandGroup(
                new RunSetPivotAngle(pivot, 47.18),
                new RunSetExtendPosition(extend, 52000),
                new RunSetFlexAngle(wrist, -22.7)
        ));
        eventMap.put(
            "High Goal Short End",
            new ParallelCommandGroup(
                new RunSetPivotAngle(pivot, 152),
                new RunSetExtendPosition(extend, 38250),
                new RunSetFlexAngle(wrist, -35)
        ));
        eventMap.put(
            "Medium Goal Long End",
            new ParallelCommandGroup( 
                new RunSetPivotAngle(pivot, 47),
                new RunSetExtendPosition(extend, 7292),
                new RunSetFlexAngle(wrist, -28)
        ));
        eventMap.put(
            "Medium Goal Short End",
            new ParallelCommandGroup( 
                new RunSetPivotAngle(pivot, 189),
                new RunSetExtendPosition(extend, 0),
                new RunSetFlexAngle(wrist, -62)
        )); 
        eventMap.put(
            "Shelf Pickup",
            new ParallelCommandGroup(
                new RunSetPivotAngle(pivot, 61),
                new RunSetExtendPosition(extend, 0),
                new RunSetFlexAngle(wrist, -30)
        ));                                                                
        eventMap.put(
            "Ground Pickup",
            new ParallelCommandGroup(   
                new RunSetPivotAngle(pivot, 6),
                new RunSetExtendPosition(extend, 0),
                new RunSetFlexAngle(wrist, -18)
        ));

        // full commands 
        eventMap.put(
            "Score High Goal Long End",
            new ParallelCommandGroup(
                new RunSetPivotAngle(pivot, 47.18),
                new RunSetExtendPosition(extend, 52000),
                new RunSetFlexAngle(wrist, -22.7)
        )
        .andThen(new WaitCommand(1))
        .andThen(new RunSetClaw(claw, true)));
    }

    private void configAutos() {
        autoManager.addDefaultOption(
            "None",     
            new PrintCommand("No Auto"));
        for(String name : autos) {
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
        // primary controller 
        primaryJoystickXButton.onTrue(eventMap.get("Stow"));
        primaryJoystickYButton.onTrue(new InstantCommand(() -> drivetrain.zeroYaw()));
        primaryJoystickAButton.onTrue(new InstantCommand(() -> drivetrain.setDriveMode(DriveMode.ROBOT_ORIENTED_SWERVE)));
        primaryJoystickBButton.onTrue(new InstantCommand(() -> drivetrain.setDriveMode(DriveMode.FIELD_ORIENTED_SWERVE)));

        primaryJoystickPOVNorth.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.SLOW)));
        primaryJoystickPOVEast.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.KINDA_SLOW)));
        primaryJoystickPOVSouth.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.NORMAL)));
        primaryJoystickPOVWest.onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(SpeedMode.FAST)));

        // claw
        secondaryJoystickAButton.onTrue(eventMap.get("Toggle Claw"));

        // presets
        secondaryJoystickBButton.onTrue(eventMap.get("Stow"));
        secondaryJoystickPOVNorth.onTrue(eventMap.get("High Goal Long End"));
        secondaryJoystickYButton.onTrue(eventMap.get("High Goal Short End"));
        secondaryJoystickPOVEast.onTrue(eventMap.get("Medium Goal Long End"));
        secondaryJoystickXButton.onTrue(eventMap.get("Medium Goal Short End")); 
        secondaryJoystickPOVWest.onTrue(eventMap.get("Shelf Pickup"));                                                                
        secondaryJoystickPOVSouth.onTrue(eventMap.get("Ground Pickup"));
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
        }

        drivetrain.getModules()[0].setOffsetAngle(SmartDashboard.getNumber("Swerve Module [1] Offset", drivetrain.getModules()[0].getOffsetAngle()));
        drivetrain.getModules()[1].setOffsetAngle(SmartDashboard.getNumber("Swerve Module [2] Offset", drivetrain.getModules()[1].getOffsetAngle()));
        drivetrain.getModules()[2].setOffsetAngle(SmartDashboard.getNumber("Swerve Module [3] Offset", drivetrain.getModules()[2].getOffsetAngle()));
        drivetrain.getModules()[3].setOffsetAngle(SmartDashboard.getNumber("Swerve Module [4] Offset", drivetrain.getModules()[3].getOffsetAngle()));

        drivetrain.getModules()[0].setInverted(SmartDashboard.getBoolean("Swerve Module [1] Offset", drivetrain.getModules()[0].getInverted()));
        drivetrain.getModules()[1].setInverted(SmartDashboard.getBoolean("Swerve Module [2] Offset", drivetrain.getModules()[1].getInverted()));
        drivetrain.getModules()[2].setInverted(SmartDashboard.getBoolean("Swerve Module [3] Offset", drivetrain.getModules()[2].getInverted()));
        drivetrain.getModules()[3].setInverted(SmartDashboard.getBoolean("Swerve Module [4] Offset", drivetrain.getModules()[3].getInverted()));

        // pivot
        SmartDashboard.putNumber("Pivot Setpoint", pivot.getSetpoint());
        SmartDashboard.putNumber("Pivot Position", pivot.getAngle());

        pivot.setOffsetAngle(SmartDashboard.getNumber("Pivot Offset", pivot.getOffsetAngle()));

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
