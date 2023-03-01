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
    private double autoGyroInit = 0; // for maintaining gyro position after auto

    private final Odometer odometer = new Odometer(); // Custom odometer that works for Holonomic Swerve
    private double prevTimeSeconds = -1; // for updating odometry

    private double limelightYaw = 0.0;
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
                System.out.println("lmao");
            }
        }).start();
    }

    // Zero the heading of the gyro (Sets to 0)
    public void zeroHeading() {
        gyro.reset();
    }

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
    public void updateOdometry() {
        // Speeds of the wheels in meters per second
        double frontLeftVelocity = frontLeft.getDriveVelocity();
        double frontRightVelocity = frontRight.getDriveVelocity();
        double backRightVelocity = backRight.getDriveVelocity();
        double backLeftVelocity = backLeft.getDriveVelocity();

        // Angles of the wheels [0, 360)
        double frontLeftAngle = frontLeft.getAngle() + getHeading();
        double frontRightAngle = frontRight.getAngle() + getHeading();
        double backRightAngle = backRight.getAngle() + getHeading();
        double backLeftAngle = backLeft.getAngle() + getHeading();

        // The vector components of the wheels, based on their current values
        double frontLeftX = MathUtils.getHeadingX(frontLeftAngle) * frontLeftVelocity;
        double frontLeftY = MathUtils.getHeadingY(frontLeftAngle) * frontLeftVelocity;
        double frontRightX = MathUtils.getHeadingX(frontRightAngle) * frontRightVelocity;
        double frontRightY = MathUtils.getHeadingY(frontRightAngle) * frontRightVelocity;
        double backRightX = MathUtils.getHeadingX(backRightAngle) * backRightVelocity;
        double backRightY = MathUtils.getHeadingY(backRightAngle) * backRightVelocity;
        double backLeftX = MathUtils.getHeadingX(backLeftAngle) * backLeftVelocity;
        double backLeftY = MathUtils.getHeadingY(backLeftAngle) * backLeftVelocity;

        // Calculate the odometry vector components [-maxDriveVel, maxDriveVel]
        double velocityX = (frontLeftX + frontRightX + backRightX + backLeftX) / 4.0;
        double velocityY = (frontLeftY + frontRightY + backRightY + backLeftY) / 4.0;
        
        // Calculate the period in seconds since last update
        double currTimeSec = WPIUtilJNI.now() * 1.0e-6;
        double period = prevTimeSeconds >= 0 ? currTimeSec - prevTimeSeconds : 0.0;
        prevTimeSeconds = currTimeSec;

        // velocityX is the distance traveled in meters in a second, then multiplied by the period
        double deltaY = velocityX * period;
        double deltaX = velocityY * period;
        odometer.update(deltaX, deltaY);
    }

    @Override
    public void periodic() {
        // Update the odometry, using our own vector-based odometry for Holonomic Swerve
        updateOdometry();
        // TalonSRX srx = ((SwerveModuleProto) leftFront).getSteer();
        // SmartDashboard.putNumber("LF Steer Pos", srx.getSelectedSensorPosition());
        // double output = srx.getStatorCurrent();
        // SmartDashboard.putNumber("LF Steer Curr", output);

        CANSparkMax spark = ((SwerveModule) frontLeft).getDrive();
        SmartDashboard.putNumber("LF Drive Pos", spark.getEncoder().getPosition());
        double output = spark.getOutputCurrent();
        SmartDashboard.putNumber("LF Drive Curr", output);
        if (output >= 30D) 
            spark.getEncoder().setPosition(0);

        frontLeft.setOffsetAngle(SmartDashboard.getNumber("Swerve[1] Offset Degrees", frontLeft.getOffsetAngle()));
        frontRight.setOffsetAngle(SmartDashboard.getNumber("Swerve[2] Offset Degrees", frontRight.getOffsetAngle()));
        backRight.setOffsetAngle(SmartDashboard.getNumber("Swerve[3] Offset Degrees", backRight.getOffsetAngle()));
        backLeft.setOffsetAngle(SmartDashboard.getNumber("Swerve[4] Offset Degrees", backLeft.getOffsetAngle()));
    }

    /**
     * Invoke stop() on all modules so the robot stops
     */
    public void stopModules() {
        for(SwerveModule module : modules)
            module.stop();
    }

    /**
     * @param desiredStates The states to set the modules to, in the order specified
     *                      in kinematics
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        for(int i = 0; i < 4; i++) {
            // Optimize States (Both Teleop and Auto gives unoptimized)
            SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredStates[i], Rotation2d.fromDegrees(modules[i].getAngle()));
            modules[i].setDesiredState(optimizedState);
        }
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

    public void setLimelightYaw(double yaw) {
        limelightYaw = yaw;
    }

    public double getLimelightYaw() {
        return limelightYaw;
    }

    public boolean isLockDriveWhileTargeting() {
        return lockDriveWhileTargeting;
    }

    public void setLockDriveWhileTargeting(boolean b) {
        lockDriveWhileTargeting = b;
    }
}