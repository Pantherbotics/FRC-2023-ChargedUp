package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule( //1
        DriveConstants.kFrontLeftModuleID, 
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftTurningEncoderPort,
        DriveConstants.kFrontLeftTurningEncoderOffsetDeg 
    );
    private final SwerveModule frontRight = new SwerveModule( //2
        DriveConstants.kFrontRightModuleID,
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightTurningEncoderPort,
        DriveConstants.kFrontRightTurningEncoderOffsetDeg
    );
    private final SwerveModule backRight = new SwerveModule( //3
        DriveConstants.kBackRightModuleID,
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightTurningEncoderPort,
        DriveConstants.kBackRightTurningEncoderOffsetDeg
    );
    private final SwerveModule backLeft = new SwerveModule( //4
        DriveConstants.kBackLeftModuleID,
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort, 
        DriveConstants.kBackLeftTurningEncoderPort,
        DriveConstants.kBackLeftTurningEncoderOffsetDeg
    );                                                            //     1          2           3          4
    private final SwerveModule[] swerveModules = new SwerveModule[] {frontLeft, frontRight, backRight, backLeft};

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry( //WPILib's odometry class is fine, I'm not making a new one
        DriveConstants.kDriveKinematics, 
        new Rotation2d(0), //At this point the gyro has not been reset but all good we can just pass in 0 degrees by making a new Rotation2d(0)
        getModulePositions()
    );

    private DriveMode driveMode = DriveMode.SWERVE; //By default it drives relative to the robot, not the field

    public Drivetrain() {
        //Zero the gyro after 1 second while it calibrates
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {}
        }).start();
    }

    //Zero the heading of the gyro (Sets to 0)
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Get the rotation of the robot (positive CCW, negative CW)
     * @return the current heading of the robot in degrees [-180, 180]
     */
    public double getHeading() {
        return -gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * @return The current pose of the robot
     * A Pose2d is essentially a container for a Translation2d (an x and a y) and a Rotation2d
     */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /**
     * Reset the odometer to the specified pose
     * @param pose The new pose
     */
    public void resetOdometry(Pose2d pose) {
        //When the auto starts it will reset the odometry. If the robot's rotation isn't 0 at the start, configure the gyro
        // to report correct values for the rest of the match.
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        //Update the odometry
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putString("Location", getPose().getTranslation().toString());
    }

    /**
     * A convenient way of representing the ways the robot can be controlled
     * As of 1/16/23 there are only two modes but more can be added if desired (West coast, tank, etc.)
     */
    public static enum DriveMode {
        FIELD_ORIENTED_SWERVE,
        SWERVE;
        
        public String toString() {
            //switch statements are goated
            switch(this) {
                case FIELD_ORIENTED_SWERVE:
                    return "Field Oriented Swerve";
                case SWERVE:
                    return "Swerve";
                default:
                    return "";
            }
        }
    }

    /**
     * Sets the drive mode
     * @param driveMode The desired mode to put the drivetrain in
     */
    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    /**
     * @return The current drive mode of the drivetrain
     */
    public DriveMode getDriveMode() {
        return driveMode;
    }

    /**
     * @return The actual swerve modules objects, in the order specified in kinematics
     */
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    /**
     * Invokes stop() on all modules so the robot stops
     */
    public void stopModules() {
        for(int i = 0; i < 4; i++)
            swerveModules[i].stop();
    }

    /**
     * @return An array of the current positions of the modules, in the order specified in kinematics
     * A SwerveModulePosition is like a SwerveModuleState except that it contains
     * the wheel's measured distance rather than its velocity whatever the fuck that means
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++)
            modulePositions[i] = swerveModules[i].getPosition();
        return modulePositions;
    }

    /**
     * @return An array of the current states of the modules, in the order specified in kinematics
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++)
            moduleStates[i] = swerveModules[i].getState();
        return moduleStates;
    }
    /**
     * @param desiredStates The states to set the modules to, in the order specified in kinematics
     * This method does not optimize the states beforehand, should be done before passing in the states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        for(int i = 0; i < 4; i++)
            swerveModules[i].setDesiredState(desiredStates[i]);
    }
}