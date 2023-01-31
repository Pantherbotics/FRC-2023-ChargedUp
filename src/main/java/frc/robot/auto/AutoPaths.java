package frc.robot.auto;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;

public class AutoPaths {
    //subsystems
    private final Drivetrain drivetrain;

    private HashMap<String, Command> trajectories = new HashMap<>();

    public AutoPaths(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        createTrajectories();
    }

    private void createTrajectories() {
        //trajectories.add(new NamedCommand("poggers", ));
    }

    public HashMap<String, Command> getTrajectories() {
        return trajectories;
    }
}
