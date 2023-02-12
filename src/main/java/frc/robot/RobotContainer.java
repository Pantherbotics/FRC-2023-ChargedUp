package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.commands.RunPivotArm;
import frc.robot.commands.RunSwerveJoystick;
import frc.robot.commands.RunWrist;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.swerve.Drivetrain;

public class RobotContainer {
    //Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Limelight limelight = new Limelight();
    private final Arm arm = new Arm();
    private final Wrist wrist = new Wrist();

    private final AutoPaths autoPaths = new AutoPaths(drivetrain);

    //Sendable choosers
    private final SendableChooser<Double> speedChooser = new SendableChooser<Double>();
    private final SendableChooser<String> driveModeChooser = new SendableChooser<String>();
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    //Joysticks
    private final Joystick primaryJoystick = new Joystick(OIConstants.kPrimaryJoystickID);
    private final Joystick secondaryJoystick = new Joystick(OIConstants.kSecondaryJoystickID);

    //Primary controller buttons (logitech controller)
    private final JoystickButton primaryJoystickAButton           = new JoystickButton(primaryJoystick, 1); //A Button
    private final JoystickButton primaryJoystickBButton           = new JoystickButton(primaryJoystick, 2); //B Button
    private final JoystickButton primaryJoystickXButton           = new JoystickButton(primaryJoystick, 3); //X Button
    private final JoystickButton primaryJoystickYButton           = new JoystickButton(primaryJoystick, 4); //Y Button
    private final JoystickButton primaryJoystickLeftBumperButton  = new JoystickButton(primaryJoystick, 5); //Left Bumper Button
    private final JoystickButton primaryJoystickRightBumperButton = new JoystickButton(primaryJoystick, 6); //Right Bumper Button
    private final JoystickButton primaryJoystickBackButton        = new JoystickButton(primaryJoystick, 7); //Back Button
    private final JoystickButton primaryJoystickStartButton       = new JoystickButton(primaryJoystick, 8); //Start Button
    private final JoystickButton primaryJoystickLeftStickButton   = new JoystickButton(primaryJoystick, 9); //Left Stick Button
    private final JoystickButton primaryJoystickRightStickButton  = new JoystickButton(primaryJoystick, 10); //Right Stick Button

    private final POVButton primaryJoystickPOVNorth = new POVButton(primaryJoystick, 0);   //North
    private final POVButton primaryJoystickPOVEast  = new POVButton(primaryJoystick, 90);  //East
    private final POVButton primaryJoystickPOVSouth = new POVButton(primaryJoystick, 180); //South
    private final POVButton primaryJoystickPOVWest  = new POVButton(primaryJoystick, 270); //West

    //Primary controller buttons (ps4 controller)
    private final JoystickButton secondaryJoystickSquareButton        = new JoystickButton(secondaryJoystick, 1);  //Square Button
    private final JoystickButton secondaryJoystickXButton             = new JoystickButton(secondaryJoystick, 2);  //X Button
    private final JoystickButton secondaryJoystickCircleButton        = new JoystickButton(secondaryJoystick, 3);  //Circle Button
    private final JoystickButton secondaryJoystickTriangleButton      = new JoystickButton(secondaryJoystick, 4);  //Triangle Button
    private final JoystickButton secondaryJoystickRightBumperButton   = new JoystickButton(secondaryJoystick, 6);  //Right Bumper Button
    private final JoystickButton secondaryJoystickRightTriggerButton  = new JoystickButton(secondaryJoystick, 8);  //Right Trigger Button
    private final JoystickButton secondaryJoystickLeftBumperButton    = new JoystickButton(secondaryJoystick, 5);  //Left Bumper Button
    private final JoystickButton secondaryJoystickLefTriggerButton    = new JoystickButton(secondaryJoystick, 7);  //Left Trigger Button
    private final JoystickButton secondaryJoystickShareButton         = new JoystickButton(secondaryJoystick, 9);  //Share Button
    private final JoystickButton secondaryJoystickOptionsButton       = new JoystickButton(secondaryJoystick, 10); //Options Button
    private final JoystickButton secondaryJoystickLeftJoystickButton  = new JoystickButton(secondaryJoystick, 11); //Left Joystick Button
    private final JoystickButton secondaryJoystickRightJoystickButton = new JoystickButton(secondaryJoystick, 12); //Right Joystick Button
    private final JoystickButton secondaryJoystickPlayStationButton   = new JoystickButton(secondaryJoystick, 13); //PlayStation Button
    private final JoystickButton secondaryJoystickBigButton           = new JoystickButton(secondaryJoystick, 14); //Big (Center) Button

    private final POVButton secondaryJoystickPOVNorth = new POVButton(secondaryJoystick, 0);   //North
    private final POVButton secondaryJoystickPOVEast  = new POVButton(secondaryJoystick, 90);  //East
    private final POVButton secondaryJoystickPOVSouth = new POVButton(secondaryJoystick, 180); //South
    private final POVButton secondaryJoystickPOVWest  = new POVButton(secondaryJoystick, 270); //West

    public RobotContainer() {
        configSendables();
        configButtonBindings();
        updateSmartDashboard();
    }

    private void configSendables() {
        //speed chooser
        speedChooser.setDefaultOption("Slow", 0.25);
        speedChooser.addOption("Normal", 0.65);
        speedChooser.addOption("Demon", 1.0);
        SmartDashboard.putData("Speed", speedChooser);

        //drive mode chooser
        driveModeChooser.setDefaultOption("Robot Oriented", "Swerve");
        driveModeChooser.addOption("Field Oriented", "Field Oriented Swerve");
        driveModeChooser.addOption("Boat", "Boat");
        driveModeChooser.addOption("Car", "Car");
        driveModeChooser.addOption("West Coast", "West Coast");
        driveModeChooser.addOption("Tank", "Tank");
        SmartDashboard.putData("Drive Mode", driveModeChooser);

        //auto chooser
        autoChooser.setDefaultOption("None", null);
        for(Map.Entry<String, Command> traj : autoPaths.getTrajectories().entrySet())
        {
            if(!traj.getKey().equals("None"))
                autoChooser.addOption(traj.getKey(), traj.getValue());
        }
        SmartDashboard.putData(autoChooser);
    }

    private void configButtonBindings() {
        //the drivetrain obviously needs to drive by default
        drivetrain.setDefaultCommand(new RunSwerveJoystick(
            drivetrain, 
            primaryJoystick, 
            speedChooser::getSelected, 
            driveModeChooser::getSelected
        ));

        //wrist manual control
        wrist.setDefaultCommand(new RunWrist(
            wrist, 
            secondaryJoystick
        ));

        secondaryJoystickRightBumperButton.toggleOnTrue(new RunPivotArm(arm, true));
        secondaryJoystickLeftBumperButton.toggleOnTrue(new RunPivotArm(arm, false));

        //TODO: add more commands
    }

    public void updateSmartDashboard() {  
        //TODO: add more info
    }

    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }
}
