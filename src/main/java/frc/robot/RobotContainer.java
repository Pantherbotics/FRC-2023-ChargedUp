package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunChooseDriveMode;
import frc.robot.commands.RunClaw;
import frc.robot.commands.RunSwerveJoystick;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.Drivetrain.DriveMode;

public class RobotContainer {
    //subsystems
    public final Drivetrain drivetrain = new Drivetrain();
    public final Claw claw = new Claw();
    //public final Arm arm = new Arm();

    public final SendableChooser<Double> speedChooser;

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

    
    
    public RobotContainer(Robot robot) {
        this.speedChooser = robot.speedChooser;
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //the drivetrain obviously needs to drive by default
        drivetrain.setDefaultCommand(new RunSwerveJoystick(
            drivetrain, 
            primaryJoystick, 
            speedChooser::getSelected, 
            drivetrain::getDriveMode
        ));

        //toggle between field oriented and normal
        primaryJoystickPOVEast.toggleOnTrue(new RunChooseDriveMode(drivetrain, DriveMode.FIELD_ORIENTED_SWERVE));
        primaryJoystickPOVEast.toggleOnTrue(new RunChooseDriveMode(drivetrain, DriveMode.SWERVE));

        //claw default manual control
        claw.setDefaultCommand(new RunClaw(
            claw,
            secondaryJoystick
        ));

        //pid testing
        secondaryJoystickTriangleButton.toggleOnTrue(new InstantCommand(() ->
            claw.setDoPID(!claw.getDoPID())
        ));

        //TODO: add more commands
    }

    public void updateSmartDashboard() {  
        //TODO: add more info
    }
}
