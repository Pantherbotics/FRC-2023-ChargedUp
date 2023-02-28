package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.DriveMode;
import frc.robot.subsystems.swerve.Drivetrain;

import java.util.function.Supplier;

import static frc.robot.util.MathUtils.powAxis;

public class RunSwerveJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final Joystick joystick;
    private final Supplier<Double> speedChooser;
    private final Supplier<DriveMode> driveModeChooser;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public RunSwerveJoystick(Drivetrain drivetrain, Joystick joystick, Supplier<Double> speedChooser, Supplier<DriveMode> driveModeChooser) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        this.speedChooser = speedChooser;
        this.driveModeChooser = driveModeChooser;

        // These limiters help to smooth out the joystick input by limiting the
        // acceleration during sudden changes
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch(driveModeChooser.get()) {
            case FIELD_ORIENTED_SWERVE:
                runSwerve(true);
            case SWERVE:
                runSwerve(false);
            case BOAT:
                runBoat();
            case CAR:
                runCar();
            case WEST_COAST:
                runWestCoast(); 
            case TANK:
                runTank();
            default:
                break;
        }
    }

    private void runSwerve(boolean fieldOriented) {
        // Background on the speed values:
        // - YL is positive when pulled BACKWARDS, and negative when pushed FORWARDS
        // (not intuitive)
        // - XL is positive when pushed to the right, and negative when pushed to the
        // left (normal)
        // - The Y axis on the joystick should control X (forward/backward) movement of
        // the robot, and vice versa
        // - In order for the Y axis (negative when forward) to drive X forward
        // (positive), it needs to be negated
        // - In order for the X axis to control the expected Y axis movement of positive
        // to the left, it is also negated
        // - XR is positive when pushed to the right, and negative when pushed to the
        // left (normal)
        // - In order for XR to follow the positive CCW of the gyro, it needs to be
        // negated

        // 1. Get real-time joystick inputs, converted to work with Swerve and WPI
        double xLeftAxis = joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftXAxisID);
        double yLeftAxis = joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID);
        double xRightAxis = joystick.getRawAxis(OIConstants.kPrimaryJoystickRightXAxisID);

        double xSpeed, ySpeed, turningSpeed;
        double targetInfluence = drivetrain.getLimelightYaw() / 27; // Limelight v1 Yaw ranges [-27, 27]

        if(drivetrain.isLockDriveWhileTargeting()) {
            xSpeed = 0;
            ySpeed = 0;
            turningSpeed = targetInfluence; // 45 degree field of view maybe
        } else {
            xSpeed = -powAxis(yLeftAxis, OIConstants.kDriverExp) * speedChooser.get();
            ySpeed = -powAxis(xLeftAxis, OIConstants.kDriverExp) * speedChooser.get();
            turningSpeed = -xRightAxis * (speedChooser.get() / 2.0) + targetInfluence;
        }

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds speeds = fieldOriented ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Rotation2d.fromDegrees(drivetrain.getHeading())) : 
            new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        drivetrain.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    private void runBoat() {
        double YL = -getLeftYAxis(); // We need to invert the Y axis so that positive is forwards
        double XR = getRightXAxis();

        // Right stick speed
        double speed = (YL * YL);// square the speed but keep the sign so it can reverse
        if (YL < 0) speed = -speed;
        if (Math.abs(speed) > 1) speed /= Math.abs(speed); // Should have the same effect as previous code.
        speed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond; // Scale it up to m/s

        double targetAngle = XR * 90;
        drivetrain.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(speed, Rotation2d.fromDegrees(0)), //left front
            new SwerveModuleState(speed, Rotation2d.fromDegrees(0)), //right front
            new SwerveModuleState(speed, Rotation2d.fromDegrees(-targetAngle)), //right back
            new SwerveModuleState(speed, Rotation2d.fromDegrees(-targetAngle)) //left back
        });
    }

    private void runCar() {
        double YL = -getLeftYAxis(); // We need to invert the Y axis so that positive is forwards
        double XR = -getRightXAxis(); // The swerve follows positive CCW wheel angles, so to turn the wheel left we
                              // must have a positive XR
        double speed = (YL * YL);// square the speed but keep the sign so it can reverse
        if (YL < 0) speed = -speed;
        if(Math.abs(speed) > 1) speed /= Math.abs(speed);
        speed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond; // Scale it up to m/s

        double targetAngle = XR * 90;
        drivetrain.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(speed, Rotation2d.fromDegrees(targetAngle)), //front left
            new SwerveModuleState(speed, Rotation2d.fromDegrees(targetAngle)), //front right
            new SwerveModuleState(speed, Rotation2d.fromDegrees(0)),  //back right
            new SwerveModuleState(speed, Rotation2d.fromDegrees(0)) //back left
        });
    }

    private void runWestCoast() {
        // No negation of these values since the west coast code we use is already
        // handling the inverted Y axis
        double YL = getLeftYAxis();
        double XR = getRightXAxis();

        // Y axis weirdness handled here already
        double left = (XR - YL) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double right = (-XR - YL) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        
        drivetrain.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(left, Rotation2d.fromDegrees(0)), //left front
            new SwerveModuleState(right, Rotation2d.fromDegrees(0)), //right front
            new SwerveModuleState(right, Rotation2d.fromDegrees(0)), //right back
            new SwerveModuleState(left, Rotation2d.fromDegrees(0)) //left back
        });
    }

    private void runTank() {
        double YL = -getLeftYAxis(); // Invert the Y axis so that positive is forwards
        double YR = -getRightYAxis(); // Invert the Y axis so that positive is forwards

        double left = YL * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double right = YR * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

        drivetrain.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(left, Rotation2d.fromDegrees(0)), //left front
            new SwerveModuleState(right, Rotation2d.fromDegrees(0)), //right front
            new SwerveModuleState(right, Rotation2d.fromDegrees(0)), //right back
            new SwerveModuleState(left, Rotation2d.fromDegrees(0)) //left back
        });
    }   

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * @return The X axis value of the left joystick. It should be positive to the right.
     */
    private double getLeftXAxis() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftXAxisID);
    }

    /**
     * @return The Y axis value of the left joystick. It should be NEGATIVE when forwards.
     */
    private double getLeftYAxis() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID);
    }

    /**
     * @return The X axis value of the right joystick. It should be positive to the right.
     */
    private double getRightXAxis() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickRightXAxisID);
    }

    /**
     * @return The Y axis value of the right joystick. It should be NEGATIVE when forwards.
     */
    private double getRightYAxis() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickRightYAxisID);
    }
}