package frc.robot.subsystems.drive.modes;

public enum SpeedMode {
    SLOW("Slow", 0.25),
    KINDA_SLOW("Kinda Slow", 0.45),
    NORMAL("Normal", 0.65),
    FAST("Fast", 0.80),
    DEMON("Demon", 1.00);

    private String name;
    private double scalar;

    SpeedMode(String name, double scalar)
    {
        this.name = name;
        this.scalar = scalar;
    }

    public double getScalar()
    {
        return scalar;
    }

    @Override
    public String toString() {
        return name;
    }
}
