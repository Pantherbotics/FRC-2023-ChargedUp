package frc.robot.subsystems.drive.modes;

public enum DriveMode {
    ROBOT_ORIENTED_SWERVE("Robot Oriented"),
    FIELD_ORIENTED_SWERVE("Field Oriented"),
    BOAT("Boat"),
    CAR("Car"),
    WEST_COAST("West Coast"),
    TANK("Tank");

    private String name;

    DriveMode(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
    }
}
