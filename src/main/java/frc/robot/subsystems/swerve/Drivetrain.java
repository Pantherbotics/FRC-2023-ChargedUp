package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule frontLeft, frontRight, backRight, backLeft;

    private final AHRS gyro;
    private final SwerveDriveOdometry odometer;
    

    public Drivetrain() {
        frontLeft = new SwerveModule( //1
            ModuleConstants.kFrontLeftModuleID, 
            ModuleConstants.kFrontLeftDriveMotorPort,
            ModuleConstants.kFrontLeftTurningMotorPort,
            ModuleConstants.kFrontLeftTurningEncoderPort,
            ModuleConstants.kFrontLeftTurningEncoderOffsetDeg 
        );
        frontRight = new SwerveModule( //2
            ModuleConstants.kFrontRightModuleID,
            ModuleConstants.kFrontRightDriveMotorPort,
            ModuleConstants.kFrontRightTurningMotorPort, 
            ModuleConstants.kFrontRightTurningEncoderPort,
            ModuleConstants.kFrontRightTurningEncoderOffsetDeg
        );
        backRight = new SwerveModule( //3
            ModuleConstants.kBackRightModuleID,
            ModuleConstants.kBackRightDriveMotorPort,
            ModuleConstants.kBackRightTurningMotorPort, 
            ModuleConstants.kBackRightTurningEncoderPort,
            ModuleConstants.kBackRightTurningEncoderOffsetDeg
        );
        backLeft = new SwerveModule( //4
            ModuleConstants.kBackLeftModuleID,
            ModuleConstants.kBackLeftDriveMotorPort,
            ModuleConstants.kBackLeftTurningMotorPort, 
            ModuleConstants.kBackLeftTurningEncoderPort,
            ModuleConstants.kBackLeftTurningEncoderOffsetDeg
        );

        gyro = new AHRS(SPI.Port.kMXP);
        odometer = new SwerveDriveOdometry( //WPILib's odometry class is fine, If a future person looking at this wants to make there own go ahead
            DriveConstants.kDriveKinematics, 
            new Rotation2d(0), //At this point the gyro has not been reset but all good we can just pass in 0 degrees by making a new Rotation2d(0)
            getModulePositions()
        );

        //Zero the gyro after 1 second while it calibrates
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {}
        }).start();
    }

    /**
     * 
     * @param translation The x speed and y speed in the form of a translation2d
     * @param rotation The angle of the desired 
     * @param fieldRelative Whether the robot is in field oriented drive or not
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds;
        if(fieldRelative)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getHeading());
        else
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    //Zero the heading of the gyro (Sets to 0)
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Get the rotation of the robot (positive CCW, negative CW)
     * @return the current heading of the robot in degrees [-180, 180]
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
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
        odometer.resetPosition(getHeading(), getModulePositions(), pose);
    }

    /**
     * Invokes stop() on all modules so the robot stops
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backRight.stop();
        backLeft.stop();
    }

    /**
     * @return An array of the current positions of the modules, in the order specified in kinematics
     * A SwerveModulePosition is like a SwerveModuleState except that it contains
     * the wheel's measured distance rather than its velocity whatever the fuck that means
     * This function is mostly just because the SwerveDriveOdometry needs a SwerveModulePosition
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
     * This method does not optimize the states beforehand, should be done before passing in the states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    @Override
    public void periodic() {
        //Update the odometry
        odometer.update(getHeading(), getModulePositions());
        
        frontLeft.outputTelemetry();
        frontRight.outputTelemetry();
        backRight.outputTelemetry();
        backLeft.outputTelemetry();
    }
}