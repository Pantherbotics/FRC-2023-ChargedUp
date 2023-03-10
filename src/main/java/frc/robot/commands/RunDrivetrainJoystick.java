package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.MathUtils;

public class RunDrivetrainJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final Joystick joystick;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public RunDrivetrainJoystick(Joystick joystick, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;

        // These limiters help to smooth out the joystick input by limiting the acceleration during sudden changes
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch(drivetrain.getDriveMode()) {
            case FIELD_ORIENTED_SWERVE:
                runSwerve(true);
                break;
            case ROBOT_ORIENTED_SWERVE:
                runSwerve(false);
                break;
            case BOAT:
                runBoat();
                break;
            case CAR:
                runCar();
                break;
            case WEST_COAST:
                runWestCoast();
                break;
            case TANK:
                runTank();
                break;
            default:
                break;
        }
    }

    private void runSwerve(boolean fieldOriented) {
        // Background on the speed values:
        // - YL is positive when pulled BACKWARDS, and negative when pushed FORWARDS (not normal)
        // - XL is positive when pushed to the right, and negative when pushed to the left (normal)
        // - The Y axis on the joystick should control X (forward/backward) movement of the robot, and vice versa
        // - In order for the Y axis (negative when forward) to drive X forward (positive), it needs to be negated
        // - In order for the X axis to control the expected Y axis movement of positive to the left, it is also negated
        // - XR is positive when pushed to the right, and negative when pushed to the left (normal)
        // - In order for XR to follow the positive CCW of the gyro, it needs to be negated

        // 1. Get real-time joystick inputs, converted to work with Swerve and WPI
        double leftXAxis = getXL();
        double leftYAxis = getYL();
        double rightXAxis = getXR();

        double xSpeed, ySpeed, turnSpeed;
        double speedScalar = drivetrain.getSpeedMode().getScalar();
        
        xSpeed = -MathUtils.powAxis(leftYAxis, OIConstants.kDriverExp) * speedScalar;
        ySpeed = -MathUtils.powAxis(leftXAxis, OIConstants.kDriverExp) * speedScalar;
        turnSpeed = -rightXAxis * speedScalar;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;
        turnSpeed = Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turnSpeed = turningLimiter.calculate(turnSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = fieldOriented ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, Rotation2d.fromDegrees(drivetrain.getYaw())) :
            new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        // 5. Output each module states to wheels
        drivetrain.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    private void runBoat() {
        double leftYAxis = -getYL(); // We need to invert the Y axis so that positive is forwards
        double rightXAxis = getXR();

        // square the speed but keep the sign so it can reverse
        double speed = leftYAxis * leftYAxis * (leftYAxis < 0 ? -1 : 1); 
        speed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond; // Scale it up to m/s

        // Calculate Steering Angle
        double angle = rightXAxis * (Math.PI / 4);

        drivetrain.setModuleStates(new SwerveModuleState[] { 
            new SwerveModuleState(speed, new Rotation2d(0)), //front left
            new SwerveModuleState(speed, new Rotation2d(0)), //front right
            new SwerveModuleState(speed, new Rotation2d(-angle)), //back right
            new SwerveModuleState(speed, new Rotation2d(-angle)) //back left
        });
    }

    private void runCar() {
        double leftYAxis = -getYL(); // We need to invert the Y axis so that positive is forwards
        double rightXAxis = -getXR(); // The swerve follows positive CCW wheel angles, so to turn the wheel left we must have a positive XR

        double speed = leftYAxis * leftYAxis * (leftYAxis < 0 ? -1 : 1); // square the speed but keep the sign so it can reverse
        speed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond; // Scale it up to m/s

        // Calculate Steering Angle
        double angle = rightXAxis * (Math.PI / 4);

        drivetrain.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(speed, new Rotation2d(angle)), //front left
            new SwerveModuleState(speed, new Rotation2d(angle)), //front right
            new SwerveModuleState(speed, new Rotation2d(0)), //back right
            new SwerveModuleState(speed, new Rotation2d(0)), //back left
        });
    }

    private void runWestCoast() {
        // No negation of these values since the west coast code we use is already
        // handling the inverted Y axis
        double leftYAxis = getYL();
        double rightXAxis = getXR();

        // Y axis weirdness handled here already
        double leftSpeed = rightXAxis - leftYAxis;
        double rightSpeed = -(rightXAxis + leftYAxis);

        leftSpeed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        rightSpeed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

        drivetrain.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(leftSpeed, new Rotation2d(0)),
            new SwerveModuleState(rightSpeed, new Rotation2d(0)),
            new SwerveModuleState(rightSpeed, new Rotation2d(0)),
            new SwerveModuleState(leftSpeed, new Rotation2d(0))
        });
    }

    private void runTank() {
        double leftYAxis = -getYL(); // Invert the Y axis so that positive is forwards
        double rightYAxis = -getYR(); // Invert the Y axis so that positive is forwards

        double leftSpeed = leftYAxis * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double rightSpeed = rightYAxis * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        
        drivetrain.setModuleStates(new SwerveModuleState[] { 
            new SwerveModuleState(leftSpeed, new Rotation2d(0)),
            new SwerveModuleState(rightSpeed, new Rotation2d(0)),
            new SwerveModuleState(rightSpeed, new Rotation2d(0)),
            new SwerveModuleState(leftSpeed, new Rotation2d(0))
        });
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * @return The X axis value of the left joystick. It should be positive to the
     *         right.
     */
    private double getXL() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftXAxisID);
    }

    /**
     * @return The Y axis value of the left joystick. It should be NEGATIVE when
     *         forwards.
     */
    private double getYL() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID);
    }

    /**
     * @return The X axis value of the right joystick. It should be positive to the
     *         right.
     */
    private double getXR() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickRightXAxisID);
    }

    /**
     * @return The Y axis value of the right joystick. It should be NEGATIVE when
     *         forwards.
     */
    private double getYR() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickRightYAxisID);
    }
}