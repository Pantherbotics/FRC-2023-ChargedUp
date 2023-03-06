package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Keeps track of the robot position relative to the field
 */
public class Odometer {
    private double x, y;

    public Odometer() {
        x = 0;
        y = 0;
    }

    /**
     * @param x The delta x to add to the current value
     * @param y The delta y to add to the current value
     */
    public void update(double deltaX, double deltaY) {
        x += deltaX;
        y += deltaY;
    }

    /**
     * @return The current coordinates of the robot
     */
    public Translation2d getPoseMeters() {
        return new Translation2d(x, y);
    }

    /**
     * Reset the odometry to a position
     * @param pose The Pose2d containing the new X and Y coordinates
     */
    public void resetPosition(Pose2d pose) {
        x = pose.getX();
        y = pose.getY();
    }
}