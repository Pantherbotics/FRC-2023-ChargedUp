package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class NamedAuto {
    private String name;
    private Command command;

    public NamedAuto(String name, Command command) {
        this.name = name;
        this.command = command;
    }

    public String getName() {
        return name;
    }

    public Command getCommand() {
        return command;
    }
}
