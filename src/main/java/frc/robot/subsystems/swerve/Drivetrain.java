package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

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
import frc.robot.util.MathUtils;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule frontLeft, frontRight, backRight, backLeft;
    private final SwerveModule[] modules;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Odometer odometer = new Odometer(); // Custom odometer that works for Holonomic Swerve

    private double limelightYaw = 0.0;

    public void setLimelightYaw(double y) {
        limelightYaw = y;
    }

    public double getLimelightYaw() {
        return limelightYaw;
    }

    private boolean lockDriveWhileTargeting = false;

    public Drivetrain() {
        // Positive is CCW, Negative is CW
        frontLeft = new SwerveModule(1, ModuleConstants.kFrontLeftCANCoderOffsetDeg, false); 
        frontRight = new SwerveModule(2, ModuleConstants.kFrontRightCANCoderOffsetDeg, false); 
        backRight = new SwerveModule(3, ModuleConstants.kBackRightCANCoderOffsetDeg, true);
        backLeft = new SwerveModule(4, ModuleConstants.kBackLeftCANCoderOffsetDeg, true); 
        modules = new SwerveModule[] { frontLeft, frontRight, backRight, backLeft };

        SmartDashboard.putNumber("Swerve[1] Offset Degrees", frontLeft.getOffsetAngle());
        SmartDashboard.putNumber("Swerve[2] Offset Degrees", frontRight.getOffsetAngle());
        SmartDashboard.putNumber("Swerve[3] Offset Degrees", backRight.getOffsetAngle());
        SmartDashboard.putNumber("Swerve[4] Offset Degrees", backLeft.getOffsetAngle());

        // Zero the gyro after 1 second while it calibrates
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception ignored) {
            }
        }).start();
    }

    // Zero the heading of the gyro (Sets to 0)
    public void zeroHeading() {
        gyro.reset();
    }

    public double autoGyroInit = 0;

    /**
     * Get the rotation of the robot (positive CCW, negative CW)
     * 
     * @return the current heading of the robot in degrees [-180, 180]
     */
    public double getHeading() {
        return MathUtils.boundHalfDegrees(-gyro.getYaw() + autoGyroInit);
        // return -gyro.getYaw();
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
        // When the auto starts it will reset the odometry. If the robot's rotation
        // isn't 0 at the start, configure the gyro
        // to report correct values for the rest of the match.
        autoGyroInit = rotation.getDegrees();
        odometer.resetPosition(pose);
    }

    // Update the odometry by calculating the current wheel vectors, the overall
    // odometry vector, then the amount of movement
    private double prevTimeSeconds = -1;

    public void updateOdometry() {
        // Speeds of the wheels in meters per second
        double s1 = frontLeft.getDriveVelocity();
        double s2 = frontRight.getDriveVelocity();
        double s3 = backRight.getDriveVelocity();
        double s4 = backLeft.getDriveVelocity();
        // Angles of the wheels [0, 360)
        double heading = getHeading();
        double a1 = frontLeft.getAngle() + heading;
        double a2 = frontRight.getAngle() + heading;
        double a3 = backRight.getAngle() + heading;
        double a4 = backLeft.getAngle() + heading;
        // The vector components of the wheels, based on their current values
        double X1 = MathUtils.getHeadingX(a1) * s1;
        double Y1 = MathUtils.getHeadingY(a1) * s1;
        double X2 = MathUtils.getHeadingX(a2) * s2;
        double Y2 = MathUtils.getHeadingY(a2) * s2;
        double X3 = MathUtils.getHeadingX(a3) * s3;
        double Y3 = MathUtils.getHeadingY(a3) * s3;
        double X4 = MathUtils.getHeadingX(a4) * s4;
        double Y4 = MathUtils.getHeadingY(a4) * s4;

        // Calculate the odometry vector components [-maxDriveVel, maxDriveVel]
        double oX = (X1 + X2 + X3 + X4) / 4D;
        double oY = (Y1 + Y2 + Y3 + Y4) / 4D;
        // SmartDashboard.putString("Odo Data", "oX: " + roundStr(oX, 3) + " oY: " +
        // roundStr(oY, 3));

        // Calculate the period in seconds since last update
        double currTimeSec = WPIUtilJNI.now() * 1.0e-6;
        double period = prevTimeSeconds >= 0 ? currTimeSec - prevTimeSeconds : 0.0;
        prevTimeSeconds = currTimeSec;

        // oX is the distance traveled in meters in a second, then multiplied by the
        // period
        double changeY = oX * period;
        double changeX = oY * period;
        odometer.update(changeX, changeY);
    }

    @Override
    public void periodic() {
        // Update the odometry, using our own vector-based odometry for Holonomic Swerve
        updateOdometry();
        
        frontLeft.setOffsetAngle(SmartDashboard.getNumber("Swerve[1] Offset Degrees", frontLeft.getOffsetAngle()));
        frontRight.setOffsetAngle(SmartDashboard.getNumber("Swerve[2] Offset Degrees", frontRight.getOffsetAngle()));
        backRight.setOffsetAngle(SmartDashboard.getNumber("Swerve[3] Offset Degrees", backRight.getOffsetAngle()));
        backLeft.setOffsetAngle(SmartDashboard.getNumber("Swerve[4] Offset Degrees", backLeft.getOffsetAngle()));
    }

    /**
     * Invoke stop() on all modules so the robot stops
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * @param desiredStates The states to set the modules to, in the order specified
     *                      in kinematics
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

    public boolean isLockDriveWhileTargeting() {
        return lockDriveWhileTargeting;
    }

    public AHRS getGyro() {
        return gyro;
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

    public void setLockDriveWhileTargeting(boolean b) {
        lockDriveWhileTargeting = b;
    }
}
