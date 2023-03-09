package frc.robot.oi;

import edu.wpi.first.wpilibj.DriverStation;

public class OISelector {
    private static String[] joystickNames = new String[DriverStation.kJoystickPorts];

    /**
     * @return Whether the number of connected joysticks have change since 
     * the last time this method was called.
     */
    public static boolean didJoysticksChange() {
        boolean joysticksChanged = false;

        for(int port = 0; port < DriverStation.kJoystickPorts; port++) {
            String name = DriverStation.getJoystickName(port);

            if(!name.equals(joystickNames[port])) {
                joystickNames[port] = name;
                joysticksChanged = true;
            }
        }
        return joysticksChanged;
    }

    /**
     * @return The appropriate OI objected based on the connected joysticks.
     */
    public static OI findOI() {
        int firstPort = -1; 
        int secondPort = -1;

        for(int port = 0; port < DriverStation.kJoystickPorts; port++) {
            if(DriverStation.getJoystickName(port).equals("")) continue;
            
            if(firstPort == -1) firstPort = port;
            if(secondPort == -1) secondPort = port;
        }

        if(firstPort != -1)
            return secondPort != -1 ? 
                new DualJoystickOI(firstPort, secondPort) : 
                new SingleJoystickOI(firstPort);
        return new OI() {};
    }
}
