package frc.robot.util;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;

public class AutoPaths {
    //subsystems
    private final Drivetrain drivetrain;

    private HashMap<String, Command> trajectories = new HashMap<String, Command>();

    public AutoPaths(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        createTrajectories();
    }

    private void createTrajectories() {
        trajectories.put("None", null);

        //TODO: add more trajectories w/ pp
    }

    public HashMap<String, Command> getTrajectories() {
        return trajectories;
    }
}
