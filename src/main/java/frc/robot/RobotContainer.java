package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.commands.RunPivotArm;
import frc.robot.commands.RunSetExtendPosition;
import frc.robot.commands.RunSetPivotAngle;
import frc.robot.commands.RunSetWristPosition;
import frc.robot.commands.RunSwerveJoystick;
import frc.robot.commands.RunToggleClaw;
import frc.robot.commands.RunExtendArm;
import frc.robot.commands.RunWristJoystick;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Claw;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.swerve.DriveMode;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightMode;

public class RobotContainer {
    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Limelight reflective = new Limelight("pog", LimelightMode.REFLECTIVE);
    private final Limelight apriltag = new Limelight("poggers", LimelightMode.APRILTAG);
    private final Arm arm = new Arm(); 
    private final Wrist wrist = new Wrist();
    private final Claw claw = new Claw();

    // Auto paths
    private final AutoPaths autoPaths = new AutoPaths(
        drivetrain, 
        reflective,
        apriltag, 
        arm,
        wrist,
        claw
    );

    // Choosers
    private final SendableChooser<Double> speedChooser = new SendableChooser<Double>();
    private final SendableChooser<DriveMode> driveModeChooser = new SendableChooser<DriveMode>();
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    // Joysticks
    private final Joystick primaryJoystick = new Joystick(OIConstants.kPrimaryJoystickID);
    private final Joystick secondaryJoystick = new Joystick(OIConstants.kSecondaryJoystickID);

    // Primary controller buttons 
    private final JoystickButton primaryJoystickAButton = new JoystickButton(primaryJoystick, 1); // A Button
    private final JoystickButton primaryJoystickBButton = new JoystickButton(primaryJoystick, 2); // B Button
    private final JoystickButton primaryJoystickXButton = new JoystickButton(primaryJoystick, 3); // X Button
    private final JoystickButton primaryJoystickYButton = new JoystickButton(primaryJoystick, 4); // Y Button
    private final JoystickButton primaryJoystickLeftBumperButton = new JoystickButton(primaryJoystick, 5); // Left                                                                                                                                                                                                                     // Button
    private final JoystickButton primaryJoystickRightBumperButton = new JoystickButton(primaryJoystick, 6); // Right                                                                                                                                                                                                                        // Button
    private final JoystickButton primaryJoystickBackButton = new JoystickButton(primaryJoystick, 7); // Back Button
    private final JoystickButton primaryJoystickStartButton = new JoystickButton(primaryJoystick, 8); // Start Button
    private final JoystickButton primaryJoystickLeftJoystickButton = new JoystickButton(primaryJoystick, 9); // Left                                                                                                                                                                                                                         // Button
    private final JoystickButton primaryJoystickRightJoystickButton = new JoystickButton(primaryJoystick, 10); // Right
                                                                                     
    private final POVButton primaryJoystickPOVNorth = new POVButton(primaryJoystick, 0); // North
    private final POVButton primaryJoystickPOVEast = new POVButton(primaryJoystick, 90); // East
    private final POVButton primaryJoystickPOVSouth = new POVButton(primaryJoystick, 180); // South
    private final POVButton primaryJoystickPOVWest = new POVButton(primaryJoystick, 270); // West

    // Secondary controller buttons 
    private final JoystickButton secondaryJoystickAButton = new JoystickButton(secondaryJoystick, 1); // Square Button
    private final JoystickButton secondaryJoystickBButton = new JoystickButton(secondaryJoystick, 2); // X Button
    private final JoystickButton secondaryJoystickXButton = new JoystickButton(secondaryJoystick, 3); // Circle Button
    private final JoystickButton secondaryJoystickYButton = new JoystickButton(secondaryJoystick, 4); // Triangle Button
    private final JoystickButton secondaryJoystickLeftBumperButton = new JoystickButton(secondaryJoystick, 5); // Right                                                                                                                                                                                           // Button
    private final JoystickButton secondaryJoystickRightBumperButton = new JoystickButton(secondaryJoystick, 6); // Left// Bumper                                                                                                               
    private final JoystickButton secondaryJoystickBackButton = new JoystickButton(secondaryJoystick, 7); // Share Button
    private final JoystickButton secondaryJoystickStartButton = new JoystickButton(secondaryJoystick, 8); // Options                                                                                                          
    private final JoystickButton secondaryJoystickLeftJoystickButton = new JoystickButton(secondaryJoystick, 9); // Left                                                                                                                                                                                                                                 // Button
    private final JoystickButton secondaryJoystickRightJoystickButton = new JoystickButton(secondaryJoystick, 10); // Right                                                                                                                   
                                                                                                                
    private final POVButton secondaryJoystickPOVNorth = new POVButton(secondaryJoystick, 0); // North
    private final POVButton secondaryJoystickPOVEast = new POVButton(secondaryJoystick, 90); // East
    private final POVButton secondaryJoystickPOVSouth = new POVButton(secondaryJoystick, 180); // South
    private final POVButton secondaryJoystickPOVWest = new POVButton(secondaryJoystick, 270); // West

    public RobotContainer() {
        configButtonBindings();
        configChoosers();
    }

    private void configButtonBindings() {
        // drivetrain manual control
        drivetrain.setDefaultCommand(new RunSwerveJoystick(
            drivetrain,
            primaryJoystick,
            speedChooser::getSelected,
            driveModeChooser::getSelected
        ));

        // wrist manual control
        wrist.setDefaultCommand(new RunWristJoystick(wrist, secondaryJoystick));

        // pivot manual control
        secondaryJoystickXButton.whileTrue(new RunPivotArm(arm, true));
        secondaryJoystickYButton.whileTrue(new RunPivotArm(arm, false));

        // extension manual control 
        secondaryJoystickLeftBumperButton.whileTrue(new RunExtendArm(arm, true));
        secondaryJoystickRightBumperButton.whileTrue(new RunExtendArm(arm, false));

        // claw manual control
        secondaryJoystickAButton.toggleOnTrue(new RunToggleClaw(claw));

        // zero position
        secondaryJoystickBButton.whileTrue(new SequentialCommandGroup(
            new RunSetPivotAngle(arm, ArmConstants.kPivotZeroAngle),
            new RunSetExtendPosition(arm, 0),
            new RunSetWristPosition(wrist, 0, 0)
        ));

        // high goal
        secondaryJoystickPOVNorth.whileTrue(new SequentialCommandGroup( 
            new RunSetPivotAngle(arm, 48),
            new RunSetExtendPosition(arm, 51000),
            new RunSetWristPosition(wrist, -13000, 0)
        ));
        // medium goal
        secondaryJoystickPOVEast.whileTrue(new SequentialCommandGroup( 
            new RunSetPivotAngle(arm, 48),
            new RunSetExtendPosition(arm, 0),
            new RunSetWristPosition(wrist, -13000, 0)
        ));
        // shelf
        secondaryJoystickPOVWest.whileTrue(new SequentialCommandGroup(
            new RunSetPivotAngle(arm, 70),
            new RunSetExtendPosition(arm, 0),
            new RunSetWristPosition(wrist, -13000, 0)
        ));                                                                
        // picking off ground   
        secondaryJoystickPOVSouth.whileTrue(new SequentialCommandGroup(
            new RunSetPivotAngle(arm, 10),
            new RunSetExtendPosition(arm, 0),
            new RunSetWristPosition(wrist, -17000, 0)
        ));
    }

    private void configChoosers() {
        // speed chooser
        speedChooser.setDefaultOption("Slow", 0.25);
        speedChooser.addOption("Kinda Slow", 0.45);
        speedChooser.addOption("Normal", 0.65);
        speedChooser.addOption("Demon", 1.00);
        SmartDashboard.putData("Speed", speedChooser);

        // drive mode chooser
        driveModeChooser.setDefaultOption("Robot Oriented", DriveMode.SWERVE);
        driveModeChooser.addOption("Field Oriented", DriveMode.FIELD_ORIENTED_SWERVE);
        driveModeChooser.addOption("Boat", DriveMode.BOAT);
        driveModeChooser.addOption("Car", DriveMode.CAR);
        driveModeChooser.addOption("West Coast", DriveMode.WEST_COAST);
        driveModeChooser.addOption("Tank", DriveMode.TANK);
        SmartDashboard.putData("Drive Mode", driveModeChooser);

        // auto chooser
        for(Map.Entry<String, Command> traj : autoPaths.getPaths().entrySet())
        {
            String name = traj.getKey();
            Command command = traj.getValue();
            if(name.equals("None"))
                autoChooser.setDefaultOption(name, command);
            else
                autoChooser.addOption(name, command);
        }
        SmartDashboard.putData("Auto", autoChooser);
    }

    public void updateSmartDashboard() {
        // drivetrain
        SmartDashboard.putNumber("Drivetrain Heading", drivetrain.getHeading());

        SmartDashboard.putNumber("Swerve [1] Speed", drivetrain.getFrontLeft().getDriveVelocity());
        SmartDashboard.putNumber("Swerve [1] Angle", drivetrain.getFrontLeft().getAngle());
        
        SmartDashboard.putNumber("Swerve [2] Speed", drivetrain.getFrontRight().getDriveVelocity());
        SmartDashboard.putNumber("Swerve [2] Angle", drivetrain.getFrontRight().getAngle());
        
        SmartDashboard.putNumber("Swerve [3] Speed", drivetrain.getBackRight().getDriveVelocity());
        SmartDashboard.putNumber("Swerve [3] Angle", drivetrain.getBackRight().getAngle());
        
        SmartDashboard.putNumber("Swerve [4] Speed", drivetrain.getBackLeft().getDriveVelocity());
        SmartDashboard.putNumber("Swerve [4] Angle", drivetrain.getBackLeft().getAngle());

        // limelights

        // arm
        SmartDashboard.putNumber("Arm Pivot Setpoint", arm.getPivotSetpoint());
        SmartDashboard.putNumber("Arm Pivot Position", arm.getPivotAngle());

        SmartDashboard.putNumber("Arm Extend Setpoint", arm.getExtendSetpoint());
        SmartDashboard.putNumber("Arm Extend Position", arm.getExtendPosition());

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
