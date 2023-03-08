package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.drive.modes.DriveMode;
import frc.robot.subsystems.drive.modes.SpeedMode;
import frc.robot.util.MathUtils;

public class Drivetrain extends SubsystemBase {
    //modules
    private final SwerveModule frontLeft, frontRight, backRight, backLeft;
    private final SwerveModule[] modules;

    //modes
    private DriveMode driveMode = DriveMode.ROBOT_ORIENTED_SWERVE;
    private SpeedMode speedMode = SpeedMode.SLOW;

    //gyro stuff
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private double autoGyroInit = 0;

    //odometry stuff
    private final Odometer odometer = new Odometer(); // Custom odometer that works for Holonomic Swerve
    private double prevTimeSeconds = -1;

    //vision stuff
    private double limelightYaw = 0.0;
    private boolean isLockDriveWhileTargeting = false;

    public Drivetrain() {
        frontLeft = new SwerveModule(1, ModuleConstants.kFrontLeftCANCoderOffsetDeg, false); 
        frontRight = new SwerveModule(2, ModuleConstants.kFrontRightCANCoderOffsetDeg, false); 
        backRight = new SwerveModule(3, ModuleConstants.kBackRightCANCoderOffsetDeg, true);
        backLeft = new SwerveModule(4, ModuleConstants.kBackLeftCANCoderOffsetDeg, true); 
        modules = new SwerveModule[] { frontLeft, frontRight, backRight, backLeft };

        // Zero the gyro after 1 second while it calibrates
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception ignored) {}
        }).start();
    }

    // Zero the heading of the gyro (Sets to 0)
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Get the rotation of the robot (positive CCW, negative CW)
     * @return the current heading of the robot in degrees [-180, 180]
     */
    public double getHeading() {
        return MathUtils.boundHalfDegrees(-gyro.getYaw() + autoGyroInit);
    }

    /**
     * @return the current pose of the robot
     */
    public Pose2d getPose() {
        return new Pose2d(odometer.getPoseMeters(), Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Reset the odometer to the specified pose
     * 
     * @param pose The new pose
     */
    public void resetOdometry(Rotation2d rotation, Pose2d pose) {
        // When the auto starts it will reset the odometry. If the robot's rotation isn't 0 at the start, configure the gyro
        // to report correct values for the rest of the match.
        autoGyroInit = rotation.getDegrees();
        odometer.resetPosition(pose);
    }

    // Update the odometry by calculating the current wheel vectors, the overall odometry vector, then the amount of movement
    public void updateOdometry() {
        // Speeds of the wheels in meters per second
        double frontLeftSpeed = frontLeft.getDriveVelocity();
        double frontRightSpeed = frontRight.getDriveVelocity();
        double backRightSpeed = backRight.getDriveVelocity();
        double backLeftSpeed = backLeft.getDriveVelocity();
        // Angles of the wheels [0, 360)
        double frontLeftAngle = frontLeft.getAngle() + getHeading();
        double frontRightAngle = frontRight.getAngle() + getHeading();
        double backRightAngle = backRight.getAngle() + getHeading();
        double backLeftAngle = backLeft.getAngle() + getHeading();
        // The vector components of the wheels, based on their current values
        double frontLeftX = MathUtils.getHeadingX(frontLeftAngle) * frontLeftSpeed;
        double frontLeftY = MathUtils.getHeadingY(frontLeftAngle) * frontLeftSpeed;
        double frontRightX = MathUtils.getHeadingX(frontRightAngle) * frontRightSpeed;
        double frontRightY = MathUtils.getHeadingY(frontRightAngle) * frontRightSpeed;
        double backRightX = MathUtils.getHeadingX(backRightAngle) * backRightSpeed;
        double backRightY = MathUtils.getHeadingY(backRightAngle) * backRightSpeed;
        double backLeftX = MathUtils.getHeadingX(backLeftAngle) * backLeftSpeed;
        double backLeftY = MathUtils.getHeadingY(backLeftAngle) * backLeftSpeed;

        // Calculate the odometry vector components [-maxDriveVel, maxDriveVel]
        double velocityX = (frontLeftX + frontRightX + backRightX + backLeftX) / 4.0;
        double velocityY = (frontLeftY + frontRightY + backRightY + backLeftY) / 4.0;

        // Calculate the period in seconds since last update
        double currTimeSec = WPIUtilJNI.now() * 1.0e-6;
        double period = prevTimeSeconds >= 0 ? currTimeSec - prevTimeSeconds : 0;
        prevTimeSeconds = currTimeSec;

        // oX is the distance traveled in meters in a second, then multiplied by the period
        double deltaY = velocityX * period;
        double deltaX = velocityY * period;
        odometer.update(deltaX, deltaY);
    }

    @Override
    public void periodic() {
        // Update the odometry, using our own vector-based odometry for Holonomic Swerve
        updateOdometry();
    }

    /**
     * Invoke stop() on all modules so the robot stops
     */
    public void stop() {
        for(SwerveModule module : modules)
            module.stop();
    }

    /**
     * @param desiredStates The states to set the modules to, in the order specified in kinematics
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        for (int i = 0; i < 4; i++) {
            // Optimize States (Both Teleop and Auto gives unoptimized)
            modules[i].setDesiredState(SwerveModuleState.optimize(
                desiredStates[i], 
                Rotation2d.fromDegrees(modules[i].getAngle()))
            );
        }
    }

    public SwerveModule getFrontLeft() {
        return frontLeft;
    }

    public SwerveModule getFrontRight() {
        return frontRight;
    }

    public SwerveModule getBackRight() {
        return backRight;
    }

    public SwerveModule getBackLeft() {
        return backLeft;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
    }

    public SpeedMode getSpeedMode() {
        return speedMode;
    }

    public void setSpeedMode(SpeedMode speed) {
        speedMode = speed;
    }

    public boolean getIsLockDriveWhileTargeting() {
        return isLockDriveWhileTargeting;
    }

    public void setLockDriveWhileTargeting(boolean lockDrive) {
        isLockDriveWhileTargeting = lockDrive;
    }
    
    public double getLimelightYaw() {
        return limelightYaw;
    }

    public void setLimelightYaw(double yaw) {
        limelightYaw = yaw;
    }
}
