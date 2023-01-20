package frc.robot.util;
public class Target {
    private double pitch = 0;

    private double yaw = 0;

    private double area = 0;

    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public double getArea() {
        return area;
    }

    public void setYaw(double y) {
        yaw = y;
    }

    public void setPitch(double p) {
        pitch = p;
    }

    public void setArea(double a) {
        area = a;
    }
}