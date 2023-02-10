package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.Drivetrain;

public class RunSwerveJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final Joystick joystick;
    private final Supplier<Double> speedChooser;
    private final Supplier<String> driveModeChooser;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    public RunSwerveJoystick(Drivetrain drivetrain, Joystick joystick, Supplier<Double> speedChooser, Supplier<String> driveModeChooser) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        this.speedChooser = speedChooser;
        this.driveModeChooser = driveModeChooser;

        //These limiters help to smooth out the joystick input by limiting the acceleration during sudden changes
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        switch(driveModeChooser.get()) {
            case "Swerve":
                runSwerve(false);
            case "Field Oriented Swerve":
                runSwerve(true);
            case "Boat":
                runBoat();
            case "Car":
                runCar();
            case "West Coast":
                runWestCoast();
            case "Tank":
                runTank();
            default:
                break;
        }
    }

    private void runSwerve(boolean isFieldOriented) {
        //Background on the speed values:
        // - Left Y is positive when pulled BACKWARDS, and negative when pushed FORWARDS (not intuitive)
        // - Left X is positive when pushed to the right, and negative when pushed to the left (normal)
        // - The Y axis on the joystick should control X (forward/backward) movement of the robot, and vice versa
        // - In order for the Y axis (negative when forward) to drive X forward (positive), it needs to be negated
        // - In order for the X axis to control the expected Y axis movement of positive to the left, it is also negated
        // - Right X is positive when pushed to the right, and negative when pushed to the left (normal)
        // - In order for Right X to follow the positive CCW of the gyro, it needs to be negated

        // 1. Get real-time joystick inputs, converted to work with Swerve and WPI
        double xLeftValue = joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftXAxisID);
        double yLeftValue = joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID);

        //The quirky exponent stuff is just so the value maintains its sign even after being raised by a power
        double xSpeed = Math.copySign(Math.pow(yLeftValue, OIConstants.kDriverExp), yLeftValue) * speedChooser.get();
        double ySpeed = Math.copySign(Math.pow(xLeftValue, OIConstants.kDriverExp), xLeftValue) * speedChooser.get();
        double turningSpeed = -joystick.getRawAxis(OIConstants.kPrimaryJoystickRightXAxisID) * (speedChooser.get() / 2.0);

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        drivetrain.drive(new Translation2d(xSpeed, ySpeed), turningSpeed, isFieldOriented);
    }

    private void runBoat() {
        double xRightValue = joystick.getRawAxis(OIConstants.kPrimaryJoystickRightXAxisID);
        double yLeftValue = -joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID); // We need to invert the Y axis so that positive is forwards
        
        // Right stick speed
        double speed = Math.copySign(yLeftValue * yLeftValue, yLeftValue);// square the speed but keep the sign so it can reverse
        if(Math.abs(speed) > 1) speed /= Math.abs(speed); // Should have the same effect as previous code.
        speed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond; // Scale it up to m/s

        // Calculate Steering Angl
        SwerveModuleState[] states = {
            new SwerveModuleState(speed, Rotation2d.fromDegrees(0)), //left front
            new SwerveModuleState(speed, Rotation2d.fromDegrees(0)), //right front
            new SwerveModuleState(speed, Rotation2d.fromDegrees(-xRightValue * 90)), //right back
            new SwerveModuleState(speed, Rotation2d.fromDegrees(-xRightValue * 90)) //left back
        };
        drivetrain.setModuleStates(states);
    }

    private void runCar() {
        double yLeftValue = -joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID); // We need to invert the Y axis so that positive is forwards
        double xRightValue = -joystick.getRawAxis(OIConstants.kPrimaryJoystickRightXAxisID); // The swerve follows positive CCW wheel angles, so to turn the wheel left we
                              // must have a positive XR
        double speed = Math.copySign(yLeftValue * yLeftValue, yLeftValue);// square the speed but keep the sign so it can reverse
        if(Math.abs(speed) > 1) speed /= Math.abs(speed);
        speed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond; // Scale it up to m/s

        // Calculate Steering Angle
        SwerveModuleState lF = new SwerveModuleState(speed, Rotation2d.fromDegrees(xRightValue * 90));
        SwerveModuleState rF = new SwerveModuleState(speed, Rotation2d.fromDegrees(xRightValue * 90));
        SwerveModuleState rB = new SwerveModuleState(speed, Rotation2d.fromDegrees(0));
        SwerveModuleState lB = new SwerveModuleState(speed, Rotation2d.fromDegrees(0));

        drivetrain.setModuleStates(new SwerveModuleState[] { lF, rF, rB, lB });
    }

    private void runWestCoast() {
        // No negation of these values since the west coast code we use is already
        // handling the inverted Y axis
        double YL = joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID);
        double XR = joystick.getRawAxis(OIConstants.kPrimaryJoystickRightXAxisID);

        // Y axis weirdness handled here already
        double left = (XR - YL) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double right = (-XR - YL) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        SwerveModuleState lF = new SwerveModuleState(left, Rotation2d.fromDegrees(0));
        SwerveModuleState rF = new SwerveModuleState(right, Rotation2d.fromDegrees(0));
        SwerveModuleState rB = new SwerveModuleState(right, Rotation2d.fromDegrees(0));
        SwerveModuleState lB = new SwerveModuleState(left, Rotation2d.fromDegrees(0));
        drivetrain.setModuleStates(new SwerveModuleState[] { lF, rF, rB, lB });
    }

    private void runTank() {
        double YL = -joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID); // Invert the Y axis so that positive is forwards
        double YR = -joystick.getRawAxis(OIConstants.kPrimaryJoystickRightYAxisID); // Invert the Y axis so that positive is forwards

        double left = YL * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double right = YR * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        SwerveModuleState lF = new SwerveModuleState(left, Rotation2d.fromDegrees(0));
        SwerveModuleState rF = new SwerveModuleState(right, Rotation2d.fromDegrees(0));
        SwerveModuleState rB = new SwerveModuleState(right, Rotation2d.fromDegrees(0));
        SwerveModuleState lB = new SwerveModuleState(left, Rotation2d.fromDegrees(0));
        drivetrain.setModuleStates(new SwerveModuleState[] { lF, rF, rB, lB });
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
