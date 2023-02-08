package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;

public class Limelight {
    private NetworkTable limelight;
    private NetworkTableEntry limelightTX;
    private NetworkTableEntry limelightTY;
    private NetworkTableEntry limelightTA;

    private Target target; 

    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        limelightTX = limelight.getEntry("tx");
        limelightTY = limelight.getEntry("ty");
        limelightTA = limelight.getEntry("ta");

        Notifier updateLoop = new Notifier(this::update);
        updateLoop.startPeriodic(10.0 / 1000);

        target = new Target();
    }

    public class Target {
        public boolean exists;
        public double pitch;
        public double yaw;
        public double area;
    }

    private void update() {
        target.yaw = limelightTX.getDouble(0);
        target.pitch = limelightTY.getDouble(0);
        target.area = limelightTA.getDouble(0);
    }
}
