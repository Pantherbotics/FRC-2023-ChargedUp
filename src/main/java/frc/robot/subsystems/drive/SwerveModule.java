package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.MathUtils;

public class SwerveModule {
    // Module Variables
    private final int id;
    private double offsetAngle;
    private boolean inverted;

    // Drive objects for the Module
    private final CANSparkMax drive;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    // Steering objects for the Module 
    private final TalonSRX steer;
    private final CANCoder cancoder; 

    /**
     * @param id ID of the module's motors
     * @param offsetAngle Offset of the module in degrees
     * @param inverted Whether to invert the drive motor
     */
    public SwerveModule(int id, double offsetAngle, boolean inverted) {
        this.id = id;
        this.offsetAngle = offsetAngle;
        this.inverted = inverted;

        // Create the SparkMax for the drive motor, and configure the units for its encoder
        drive = new CANSparkMax(id, MotorType.kBrushless);
        drive.restoreFactoryDefaults();
        drive.setInverted(inverted);

        driveEncoder = drive.getEncoder();
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MetersPerSec);
        driveEncoder.setPosition(0);

        // Get the Drive PID Controller and configure it for Velocity PID
        drivePID = drive.getPIDController();
        drivePID.setP(ModuleConstants.kPDrive);
        drivePID.setI(ModuleConstants.kIDrive);
        drivePID.setD(ModuleConstants.kDDrive);
        drivePID.setIZone(ModuleConstants.kIZoneDrive);
        drivePID.setFF(ModuleConstants.kFFDrive);
        drivePID.setOutputRange(-1, 1);

        // cancoder
        CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        cancoder = new CANCoder(id + 4); // Our CANCoders are configured to be IDs 5-8
        cancoder.configAllSettings(cancoderConfig);

        // talon srx
        steer = new TalonSRX(id);
        steer.configRemoteFeedbackFilter(cancoder, 0);
        steer.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 20);
        steer.config_kP(0, ModuleConstants.kPTurn);
        steer.config_kI(0, ModuleConstants.kITurn);
        steer.config_kD(0, ModuleConstants.kDTurn);
        steer.config_kF(0, ModuleConstants.kFTurn);
        steer.setSelectedSensorPosition(cancoder.getAbsolutePosition());
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    
    public double getAngle() {
        return -(MathUtils.restrictAngle(steer.getSelectedSensorPosition() * 360.0 / 4096 + offsetAngle) - 180);
    }

    public void setDesiredState(SwerveModuleState state) {
        // Ignore small states like when we let go of left stick so wheels don't default to 0 degrees
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        double target = state.angle.getDegrees();
        double errorAng = MathUtils.boundHalfDegrees(target - getAngle());
        // Error has to be negated since Positive is CCW and Negative is CW for our swerve modules
        // Convert [0, 360) in degrees to [0, 4906] in ticks (TalonSRX reads 4096 ticks from 360 degrees)
        steer.set(
            TalonSRXControlMode.Position, 
            steer.getSelectedSensorPosition() + (-errorAng) * 4096.0 / 360);
        drivePID.setReference(
                state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.neoMaxRPM,
                ControlType.kVelocity);
    }

    public void stop() {
        drivePID.setReference(0, ControlType.kVelocity);
        steer.set(ControlMode.PercentOutput, 0);
    }

    public int getID() {
        return id;
    }

    public double getOffsetAngle() {
        return offsetAngle;
    }

    public void setOffsetAngle(double angle) {
        offsetAngle = angle;
    }

    public boolean getInverted() {
        return inverted;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }
}