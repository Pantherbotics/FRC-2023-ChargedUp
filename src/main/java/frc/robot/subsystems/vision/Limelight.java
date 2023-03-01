package frc.robot.subsystems.vision;

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

    public Limelight(String name) {
        limelight = NetworkTableInstance.getDefault().getTable("limelight" + name);
        limelightTX = limelight.getEntry("tx");
        limelightTY = limelight.getEntry("ty");
        limelightTA = limelight.getEntry("ta");

        target = new Target();

        Notifier updateLoop = new Notifier(this::update);
        updateLoop.startPeriodic(10.0 / 1000);
        
        setLights(1);
    }

    private void update() {
        target.setYaw(limelightTX.getDouble(0));
        target.setPitch(limelightTY.getDouble(0));
        target.setArea(limelightTA.getDouble(0));
    }

    public Target getTarget()
    {
        return target;
    }

    public void setLights(int state) {
        limelight.getEntry("ledMode").setNumber(state);
    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    public double getDistance() {
        return 0;
    }
}
