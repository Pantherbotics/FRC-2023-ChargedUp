package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;

public class SingleJoystickOI implements OI {
    private final Joystick joystick;
     
    public SingleJoystickOI(int port) {
        joystick = new Joystick(port);
    }
}
