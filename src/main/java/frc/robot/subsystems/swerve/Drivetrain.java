package frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.stream.IntStream;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule frontLeft, frontRight, backRight, backLeft;
    private final SwerveModule[] swerveModules;

    private final AHRS gyro;
    private final SwerveDriveOdometry odometer;
    

    public Drivetrain() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        ShuffleboardLayout frontLeftLayout = tab.getLayout("Front Left", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0);
        ShuffleboardLayout frontRightLayout = tab.getLayout("Front Right", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0);
        ShuffleboardLayout backRightLayout = tab.getLayout("Back Right", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0);
        ShuffleboardLayout backLeftLayout = tab.getLayout("Back Left", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0);
        
        frontLeft = createSwerveModule( //1
            ModuleConstants.kFrontLeftDriveMotorPort,
            ModuleConstants.kFrontLeftTurnMotorPort,
            ModuleConstants.kFrontLeftCANCoderPort,
            ModuleConstants.kFrontLeftCANCoderOffsetDeg,
            frontLeftLayout        
        );
        frontRight = createSwerveModule( //2
            ModuleConstants.kFrontRightDriveMotorPort,
            ModuleConstants.kFrontRightTurnMotorPort, 
            ModuleConstants.kFrontRightCANCoderPort,
            ModuleConstants.kFrontRightCANCoderOffsetDeg,
            frontRightLayout
        );
        backRight = createSwerveModule( //3
            ModuleConstants.kBackRightDriveMotorPort,
            ModuleConstants.kBackRightTurnMotorPort, 
            ModuleConstants.kBackRightCANCoderPort,
            ModuleConstants.kBackRightCANCoderOffsetDeg,
            backRightLayout
        );
        backLeft = createSwerveModule( //4
            ModuleConstants.kBackLeftDriveMotorPort,
            ModuleConstants.kBackLeftTurnMotorPort, 
            ModuleConstants.kBackLeftCANCoderPort,
            ModuleConstants.kBackLeftCANCoderOffsetDeg,
            backLeftLayout
        );
        swerveModules = new SwerveModule[] { frontLeft, frontRight, backRight, backLeft };

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

        //Put the motors in brake mode when enabled, coast mode when disabled
        new Trigger(RobotState::isEnabled).onTrue(new StartEndCommand(() -> {
            IntStream.range(0, swerveModules.length).forEach(i -> swerveModules[i].setBrake(true));
        }, () -> {
            IntStream.range(0, swerveModules.length).forEach(i -> swerveModules[i].setBrake(false));
        }));
    }

    private SwerveModule createSwerveModule(int driveMotorPort, int turnMotorPort, int cancoderPort, double cancoderOffset, ShuffleboardContainer container) {
        return new SwerveModule(
            new SwerveSpeedController(driveMotorPort, container),
            new SwerveSteerController(turnMotorPort, cancoderPort, cancoderOffset, container)
        );
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
     * stops all 4 modules
     */
    public void stop() {
        IntStream.range(0, swerveModules.length).forEach(i -> swerveModules[i].stop());
    }

    /**
     * @return An array of the current positions of the modules, in the order specified in kinematics
     * A SwerveModulePosition is like a SwerveModuleState except that it contains
     * the wheel's measured distance rather than its velocity whatever the fuck that means
     * This function is mostly just because the SwerveDriveOdometry needs a SwerveModulePosition
     */
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(swerveModules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);

    }

    /**
     * @return An array of the current states of the modules, in the order specified in kinematics
     */
    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(swerveModules).map(module -> module.getState()).toArray(SwerveModuleState[]::new);
    }
    /**
     * @param desiredStates The states to set the modules to, in the order specified in kinematics
     * This method does not optimize the states beforehand, should be done before passing in the states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        IntStream.range(0, swerveModules.length).forEach(i -> swerveModules[i].setDesiredState(desiredStates[i]));
    }

    @Override
    public void periodic() {
        //Update the odometry
        //odometer.update(getHeading(), getModulePositions());
    }
}