package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class RunSwerveJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final Joystick joystick;
    private final Supplier<Double> speedChooser;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    public RunSwerveJoystick(Drivetrain drivetrain, Joystick joystick, Supplier<Double> speedChooser, Supplier<DriveMode> driveModeChooser) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        this.speedChooser = speedChooser;

        //These limiters help to smooth out the joystick input by limiting the acceleration during sudden changes
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        switch(drivetrain.getDriveMode()) {
            case SWERVE:
                runSwerve(false);
            case FIELD_ORIENTED_SWERVE:
                runSwerve(true);
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
        double xSpeed = (yLeftValue >= 0 ? -Math.pow(yLeftValue, OIConstants.kDriverExp) : Math.pow(-yLeftValue, OIConstants.kDriverExp)) * speedChooser.get();
        double ySpeed = (xLeftValue >= 0 ? -Math.pow(xLeftValue, OIConstants.kDriverExp) : Math.pow(-xLeftValue, OIConstants.kDriverExp)) * speedChooser.get();
        double turningSpeed = -joystick.getRawAxis(OIConstants.kPrimaryJoystickRightXAxisID) * (speedChooser.get() / 2.0);

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = isFieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, drivetrain.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        // 5. Output each module states to wheels
        drivetrain.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
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
