package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.MathUtils;

public class SwerveModule {
    // Module Variables
    private final int id;
    private double offsetDeg;

    // Drive objects for the Module
    private final CANSparkMax drive;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    // Steering objects for the Module (encoder will be analogInput OR canCoder)
    private final TalonSRX steer;
    private final CANCoder canCoder; 

    /**
     * @param id ID of the module's motors
     * @param offsetDeg Offset of the module in degrees
     */
    public SwerveModule(int id, double offsetDeg, boolean inverted) {
        this.id = id;
        this.offsetDeg = offsetDeg;

        // Create the SparkMax for the drive motor, and configure the units for its
        // encoder
        drive = new CANSparkMax(id, MotorType.kBrushless);
        drive.restoreFactoryDefaults();
        drive.setInverted(inverted);
        driveEncoder = drive.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MetersPerSec);

        // Get the Drive PID Controller and configure it for Velocity PID
        drivePID = drive.getPIDController();
        drivePID.setP(ModuleConstants.kPDrive);
        drivePID.setI(ModuleConstants.kIDrive);
        drivePID.setD(ModuleConstants.kDDrive);
        drivePID.setIZone(ModuleConstants.kIZoneDrive);
        drivePID.setFF(ModuleConstants.kFFDrive);
        drivePID.setOutputRange(-1, 1);

        // Create the Steer TalonSRX
        steer = new TalonSRX(id);// set in id ())create talon object

        // Create the encoder based on Constants.ModuleConstants.kSteerEncoderType
        // Create the CANCoder and configure it to work as the RemoteSensor0 for the
        // steer motor
        canCoder = new CANCoder(id + 4); // Our CANCoders are configured to be IDs 5-8
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        canCoder.setPositionToAbsolute();
        steer.configRemoteFeedbackFilter(canCoder, 0);
        steer.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 20);
        steer.config_kP(0, ModuleConstants.kPTurn);
        steer.config_kI(0, ModuleConstants.kITurn);
        steer.config_kD(0, ModuleConstants.kDTurn);
        steer.config_kF(0, ModuleConstants.kFTurn);
        steer.setSelectedSensorPosition(canCoder.getAbsolutePosition());
        // Reset the encoders
        resetEncoders();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        return Math.toRadians(getAngle());
    }

    public void setDesiredState(SwerveModuleState state) {
        // Ignore small states like when we let go of left stick so wheels don't default to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Log some info about the target state
        double target = state.angle.getDegrees();
        double angle = MathUtils.round(target, 1);
        double speed = MathUtils.round(state.speedMetersPerSecond, 1);
        SmartDashboard.putString("Swerve[" + id + "] Set", "Angle: " + angle + ", Speed: " + speed);

        // Either run the CanCoder logic for steering, or the AnalogInput logic
        // Get the error for the angle, assuming + error is clockwise
        double errorAng = MathUtils.boundHalfDegrees(target - getAngle());
        // Error has to be negated since Positive is CCW and Negative is CW for our swerve modules
        // Convert [0, 360) in degrees to [0, 4906] in ticks (TalonSRX reads 4096 ticks from 360 degrees)
        double pos = steer.getSelectedSensorPosition() + (-errorAng) * (4096.0 / 360);
        steer.set(TalonSRXControlMode.Position, pos);

        // Drive Speed with spark and PID (or by percent output using the 2nd line)
        drivePID.setReference(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.neoMaxRPM, ControlType.kVelocity);
        // drive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }

    public void stop() {
        drivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
        // drive.set(0);
        steer.set(ControlMode.PercentOutput, 0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public double getAngle() {
        return -(MathUtils.restrictAngle(steer.getSelectedSensorPosition() * 360.0 / 4096 + offsetDeg) - 180);
    }

    /**
     * @return the unbounded steering error, in degrees
     */
    public double getError(double target) {
        return target - getAngle();
    }

    /**
     * @return the steering error bounded to [-180, 180] degrees
     */
    public double getModifiedError(double target) {
        return MathUtils.boundHalfDegrees(getError(target)) / 180;
    }

    public CANSparkMax getDrive() {
        return drive;
    }

    public double getOffsetAngle() {
        return offsetDeg;
    }

    public void setOffsetAngle(double angle) {
        offsetDeg = angle;
    }
}
