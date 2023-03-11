package frc.robot.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoManager {
    private final HashMap<String, Command> autos = new HashMap<String, Command>();

    private Command selectedAuto = null; // by default no auto

    public AutoManager() {}

    private void addOption(String name, Command auto, boolean isDefault) {
        autos.put(name, auto);

        if(isDefault)
            selectedAuto = auto;
    }

    public void addOption(String name, Command auto) {
        addOption(name, auto, false);
    }

    public void addDefaultOption(String name, Command auto) {
        addOption(name, auto, true);
    }

    public HashMap<String, Command> getAutos() {
        return autos;
    }

    public Command getSelectedAuto() {
        return selectedAuto;
    }

    public void setSelectedAuto(Command auto) {
        selectedAuto = auto;
    }
}
