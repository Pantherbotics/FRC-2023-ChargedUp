package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two controllers */
public class DualJoystickOI implements OI {
    private final CommandJoystick driveJoystick, armJoystick;
    private final Trigger[] driveJoystickButtons, armJoystickButtons; 

    /**
     * @param firstPort The usb port of the drive controller
     * @param secondPort The usb port of the arm controller
     */
    public DualJoystickOI(int firstPort, int secondPort) {
        driveJoystick = new CommandJoystick(firstPort);
        armJoystick = new CommandJoystick(secondPort);

        driveJoystickButtons = new Trigger[13];
        armJoystickButtons = new Trigger[13];
        
        for(int i = 0; i < driveJoystickButtons.length; i++) {
            driveJoystickButtons[i] = driveJoystick.button(i);
            armJoystickButtons[i] = armJoystick.button(i);
        }
    }
}
