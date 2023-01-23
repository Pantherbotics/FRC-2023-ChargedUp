package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunChooseDriveMode;
import frc.robot.commands.RunSwerveJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;

public class RobotContainer {
    public final Drivetrain drivetrain = new Drivetrain();

    public final SendableChooser<Double> speedChooser;

    //Joysticks
    private final Joystick primaryJoystick = new Joystick(OIConstants.kPrimaryJoystickID);
    private final Joystick secondaryJoystick = new Joystick(OIConstants.kSecondaryJoystickID);

    //Primary controller buttons
    private final JoystickButton primaryJoystickSquareButton        = new JoystickButton(primaryJoystick, 1);  //Square Button
    private final JoystickButton primaryJoystickXButton             = new JoystickButton(primaryJoystick, 2);  //X Button
    private final JoystickButton primaryJoystickCircleButton        = new JoystickButton(primaryJoystick, 3);  //Circle Button
    private final JoystickButton primaryJoystickTriangleButton      = new JoystickButton(primaryJoystick, 4);  //Triangle Button
    private final JoystickButton primaryJoystickRightBumperButton   = new JoystickButton(primaryJoystick, 6);  //Right Bumper Button
    private final JoystickButton primaryJoystickRightTriggerButton  = new JoystickButton(primaryJoystick, 8);  //Right Trigger Button
    private final JoystickButton primaryJoystickLeftBumperButton    = new JoystickButton(primaryJoystick, 5);  //Left Bumper Button
    private final JoystickButton primaryJoystickLefTriggerButton    = new JoystickButton(primaryJoystick, 7);  //Left Trigger Button
    private final JoystickButton primaryJoystickShareButton         = new JoystickButton(primaryJoystick, 9);  //Share Button
    private final JoystickButton primaryJoystickOptionsButton       = new JoystickButton(primaryJoystick, 10); //Options Button
    private final JoystickButton primaryJoystickLeftJoystickButton  = new JoystickButton(primaryJoystick, 11); //Left Joystick Button
    private final JoystickButton primaryJoystickRightJoystickButton = new JoystickButton(primaryJoystick, 12); //Right Joystick Button
    private final JoystickButton primaryJoystickPlayStationButton   = new JoystickButton(primaryJoystick, 13); //PlayStation Button
    private final JoystickButton primaryJoystickBigButton           = new JoystickButton(primaryJoystick, 14); //Big (Center) Button
    private final POVButton primaryJoystickPOVNorth = new POVButton(primaryJoystick, 0);   //North
    private final POVButton primaryJoystickPOVEast  = new POVButton(primaryJoystick, 90);  //East
    private final POVButton primaryJoystickPOVSouth = new POVButton(primaryJoystick, 180); //South
    private final POVButton primaryJoystickPOVWest  = new POVButton(primaryJoystick, 270); //West

    //Secondary controller buttons
    private final JoystickButton secondaryJoystickAButton           = new JoystickButton(secondaryJoystick, 1); //A Button
    private final JoystickButton secondaryJoystickBButton           = new JoystickButton(secondaryJoystick, 2); //B Button
    private final JoystickButton secondaryJoystickXButton           = new JoystickButton(secondaryJoystick, 3); //X Button
    private final JoystickButton secondaryJoystickYButton           = new JoystickButton(secondaryJoystick, 4); //Y Button
    private final JoystickButton secondaryJoystickLeftBumperButton  = new JoystickButton(secondaryJoystick, 5); //Left Bumper Button
    private final JoystickButton secondaryJoystickRightBumperButton = new JoystickButton(secondaryJoystick, 6); //Right Bumper Button
    private final JoystickButton secondaryJoystickBackButton        = new JoystickButton(secondaryJoystick, 7); //Back Button
    private final JoystickButton secondaryJoystickStartButton       = new JoystickButton(secondaryJoystick, 8); //Start Button
    private final JoystickButton secondaryJoystickLeftStickButton   = new JoystickButton(secondaryJoystick, 9); //Left Stick Button
    private final JoystickButton secondaryJoystickRightStickButton  = new JoystickButton(secondaryJoystick, 10); //Right Stick Button

    private final POVButton secondaryJoystickPOVNorth = new POVButton(secondaryJoystick, 0);   //North
    private final POVButton secondaryJoystickPOVEast  = new POVButton(secondaryJoystick, 90);  //East
    private final POVButton secondaryJoystickPOVSouth = new POVButton(secondaryJoystick, 180); //South
    private final POVButton secondaryJoystickPOVWest  = new POVButton(secondaryJoystick, 270); //West
    
    public RobotContainer(Robot robot) {
        this.speedChooser = robot.speedChooser;
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(new RunSwerveJoystick(
            drivetrain, 
            primaryJoystick, 
            speedChooser::getSelected, 
            drivetrain::getDriveMode
        ));

        secondaryJoystickPOVEast.toggleOnTrue(new RunChooseDriveMode(drivetrain, DriveMode.FIELD_ORIENTED_SWERVE));
        secondaryJoystickPOVEast.toggleOnTrue(new RunChooseDriveMode(drivetrain, DriveMode.SWERVE));

        //TODO: add more commands
    }

    public void updateSmartDashboard() {  
        SmartDashboard.putNumber("Gyro", drivetrain.getHeading());
        SmartDashboard.putString("Mode", drivetrain.getDriveMode().toString());
        //TODO: add more info
    }
}
