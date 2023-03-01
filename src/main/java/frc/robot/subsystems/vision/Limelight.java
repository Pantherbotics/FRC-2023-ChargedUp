package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;

public class Limelight {
    private final NetworkTable limelight;
    private NetworkTableEntry tx, ty, ta;
    private double targetYaw, targetPitch, targetArea;
    private LimelightMode mode;

    public Limelight(String name, LimelightMode limelightMode) {
        limelight = NetworkTableInstance.getDefault().getTable("limelight-" + name);
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");

        targetYaw = 0;
        targetPitch = 0;
        targetArea = 0;

        mode = limelightMode;

        try(Notifier updateLoop = new Notifier(this::update)) {
            updateLoop.startPeriodic(1.0 / 100); // 10 ms
        }
        setLights(1);
    }

    private void update() {
        targetYaw = tx.getDouble(0);
        targetPitch = ty.getDouble(0);
        targetArea = ta.getDouble(0);
    }

    public void setLights(int state) {
        limelight.getEntry("ledMode").setNumber(state);
    }

    public double getDistance() {
        return 0;
    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetYaw()
    {
        return targetYaw;
    }

    public void setTargetYaw(double yaw) {
        targetYaw = yaw;
    } 

    public double getTargetPitch()
    {
        return targetPitch;
    }

    public void setTargetPitch(double pitch) {
        targetPitch = pitch;
    } 

    public double getTargetArea()
    {
        return targetArea;
    }

    public void setTargetArea(double area) {
        targetArea = area;
    } 
}
