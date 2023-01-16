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
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftModuleID,
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftTurningEncoderPort,
        DriveConstants.kFrontLeftTurningEncoderOffsetDeg 
    );
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightModuleID,
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightTurningEncoderPort,
        DriveConstants.kFrontRightTurningEncoderOffsetDeg
    );
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightModuleID,
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightTurningEncoderPort,
        DriveConstants.kBackRightTurningEncoderOffsetDeg
    );
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftModuleID,
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort, 
        DriveConstants.kBackLeftTurningEncoderPort,
        DriveConstants.kBackLeftTurningEncoderOffsetDeg
    );

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, 
        new Rotation2d(0), //At this point the gyro has not been reset but all good we can just pass in 0 degrees by making a new Rotation2d(0)
        getModulePositions()
    );
    

    private boolean isFieldOriented = false; //By default it drives relative to the robot, not the field

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
     * @return the current pose of the robot
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
     * Sets the drive mode
     * @param fieldOriented Whether the mode is field oriented or not
     * If it already in field oriented, does nothing and vice versa
     */
    public void toFieldOriented(boolean fieldOriented) {
        if(fieldOriented == isFieldOriented) return;
        isFieldOriented = fieldOriented;
    }

    /**
     * @return The true if the swerve is in field oriented mode, false if it is not
     */
    public boolean isFieldOriented() {
        return isFieldOriented;
    }

    /**
     * Invoke stop() on all modules so the robot stops
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backRight.stop();
        backLeft.stop();
    }

    /**
     * @return An array of the current positions of the modules, in the order specified in kinematics
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
        };
    }

    /**
     * @return An array of the current states of the modules, in the order specified in kinematics
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backRight.getState(),
            backLeft.getState()
        };
    }
    /**
     * @param desiredStates The states to set the modules to, in the order specified in kinematics
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }
}