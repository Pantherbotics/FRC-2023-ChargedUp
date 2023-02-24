package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModule {
    private final SwerveSpeedController driveController;
    private final SwerveSteerController turnController;

    /**
     * @param driveController
     * @param turnController
     * @param layout
     */
    public SwerveModule(SwerveSpeedController driveController, SwerveSteerController turnController, ShuffleboardLayout layout) {
        this.driveController = driveController;
        this.turnController = turnController;

        //shuffleboard shit
        layout.addNumber("Drive Position (m)", () -> getDrivePosition());
        layout.addNumber("Drive Velocity (mps)", () -> getDriveVelocity());
        layout.addNumber("Turn Angle (deg)", () -> getTurnAngle().getDegrees());
    }

    /**
     * @return The current position of the module in meters
     */
    public double getDrivePosition() {
        return driveController.getPosition();
    }

    /**
     * @return The current velocity of the module in meters per second
     */
    public double getDriveVelocity() {
        return driveController.getVelocity();
    }

    /**
     * @return The current angle on the module as a Rotation2d
     */
    public Rotation2d getTurnAngle() {
        return Rotation2d.fromDegrees(turnController.getAngle());
    }

    /**
     * @return The current position the module is in
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurnAngle());
    }

    /**
     * @return The current state the module is in
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurnAngle());
    }

    /**
     * @param state The desired state to set the module to
     */
    public void setDesiredState(SwerveModuleState state) {
        //Ignore small states like when we let go of left stick so  wheels don't default to 0 degrees
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        //state = SwerveModuleState.optimize(state, getTurnAngle());
        driveController.setVelocity(state.speedMetersPerSecond);
        turnController.setAngle(state.angle.getDegrees());
    }

    /**
     * Stops the modules
     */
    public void stop() {
        driveController.stop();
        turnController.stop();
    }

    /**
     * Sets the turn and drive motor to either Brake or Coast
     * @param brake True for Brake, false for Coast
     */
    public void setBrake(boolean brake) {
        driveController.setBrake(brake);
        turnController.setBrake(brake);
    }
}
