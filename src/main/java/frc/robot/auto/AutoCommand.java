package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommand {
    private final String name;
    private final Command command;
    
    public AutoCommand(String name) {
        this.name = name;
        command = loadCommandFromPathPlanner(name);
    }
    
    private Command loadCommandFromPathPlanner(String pathName) {
        return null;
    }

    public String getName() {
        return name;
    }

    public Command getCommand() {
        return command;
    }
}
