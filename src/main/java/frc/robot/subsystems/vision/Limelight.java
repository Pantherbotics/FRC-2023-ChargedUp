package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable limelight;
    private NetworkTableEntry tx, ty, ta;
    private LimelightMode mode;
    private Target target;

    public Limelight(String name, LimelightMode limelightMode) {
        limelight = NetworkTableInstance.getDefault().getTable("limelight-" + name);
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        
        target = new Target();

        mode = limelightMode;

        try(Notifier updateLoop = new Notifier(this::update)) {
            updateLoop.startPeriodic(1.0 / 100); // 10 ms
        }
        setLights(1);
    }

    private void update() {
        target.setYaw(tx.getDouble(0));
        target.setPitch(ty.getDouble(0));
        target.setArea(ta.getDouble(0));
    }

    public void setLights(int state) {
        limelight.getEntry("ledMode").setNumber(state);
    }

    public double getDistance() {
        return 0;
    }

    public Target getTarget() {
        return target;
    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }
}
