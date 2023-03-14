package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.drive.modes.DriveMode;
import frc.robot.subsystems.drive.modes.SpeedMode;
import frc.robot.util.MathUtils;

public class Drivetrain extends SubsystemBase {
    //modules
    private final SwerveModule[] modules;

    //gyro stuff
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    //odometry stuff
    private final Odometer odometer = new Odometer();
    private double prevTimeSeconds = -1;

    //modes
    private DriveMode driveMode = DriveMode.ROBOT_ORIENTED_SWERVE;
    private SpeedMode speedMode = SpeedMode.SLOW;

    public Drivetrain() {
        modules = new SwerveModule[] {
            new SwerveModule( //front left
                ModuleConstants.kFrontLeftModuleID, 
                ModuleConstants.kFrontLeftCANCoderOffsetDeg, 
                ModuleConstants.kFrontLeftDriveMotorInverted
            ),
            new SwerveModule( //front right
                ModuleConstants.kFrontRightModuleID,
                ModuleConstants.kFrontRightCANCoderOffsetDeg, 
                ModuleConstants.kFrontRightDriveMotorInverted
            ),
            new SwerveModule( //back right
                ModuleConstants.kBackRightModuleID, 
                ModuleConstants.kBackRightCANCoderOffsetDeg, 
                ModuleConstants.kBackRightDriveMotorInverted
            ),
            new SwerveModule( //back left
                ModuleConstants.kBackLeftModuleID, 
                ModuleConstants.kBackLeftCANCoderOffsetDeg, 
                ModuleConstants.kBackLeftDriveMotorInverted
        )};

        // Zero the gyro after 1 second while it calibrates
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroYaw();
            } catch(Exception ignored) {}
        }).start();
    }

    /**
     *  Zero the yaw (heading) of the gyro (Sets to 0)
     */ 
    public void zeroYaw() {
        gyro.reset();
    }

    /**
     * Get the heading of the robot (positive CCW, negative CW)
     * @return the current yaw of the robot in degrees [-180, 180]
     */
    public double getYaw() {
        return -gyro.getYaw();
    }

    /**
     * @return The current pitch of the robot in degrees [-180, 180]
     */
    public double getPitch() {
        return gyro.getPitch();
    }

    /**
     * @return the current pose of the robot
     */
    public Pose2d getPose() {
        return new Pose2d(odometer.getPoseMeters(), Rotation2d.fromDegrees(getYaw()));
    }

    /**
     * Reset the odometer to the specified pose
     * 
     * @param pose The new pose
     */
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose);
    }

    // Update the odometry by calculating the current wheel vectors, the overall odometry vector, then the amount of movement
    public void updateOdometry() {
        double overallVelocityX = 0; 
        double overallVelocityY = 0;

        for(SwerveModule module : modules) {
            double velocity = module.getDriveVelocity();
            double angle = module.getAngle() + getYaw();

            double velocityX = MathUtils.getHeadingX(angle) * velocity;
            double velocityY = MathUtils.getHeadingY(angle) * velocity;

            overallVelocityX += velocityX;
            overallVelocityY += velocityY;
        }

        overallVelocityX /= 4;
        overallVelocityY /= 4;

        // Calculate the period in seconds since last update
        double currTimeSec = WPIUtilJNI.now() * 1.0e-6;
        double period = prevTimeSeconds >= 0 ? currTimeSec - prevTimeSeconds : 0;
        prevTimeSeconds = currTimeSec;

        // oX is the distance traveled in meters in a second, then multiplied by the period
        double deltaY = overallVelocityX * period;
        double deltaX = overallVelocityY * period;
        odometer.update(deltaX, deltaY);
    }

    @Override
    public void periodic() {
        // Update the odometry
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
        for(int i = 0; i < 4; i++) {
            // Optimize States (Both Teleop and Auto gives unoptimized)
            modules[i].setDesiredState(SwerveModuleState.optimize(
                desiredStates[i], 
                Rotation2d.fromDegrees(modules[i].getAngle())
            ));
        }
    }

    public SwerveModule[] getModules() {
        return modules;
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
}

