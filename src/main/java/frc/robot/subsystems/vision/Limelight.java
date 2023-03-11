package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;

public class Limelight {
    private final NetworkTable limelight;
    private NetworkTableEntry tx, ty, ta;
    private Target target;

    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight-poggers");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        
        target = new Target();

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
