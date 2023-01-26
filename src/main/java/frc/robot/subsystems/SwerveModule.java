package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    //----------------------------------------------------------------------------------------------
    //  All of the following code is documented in the SwerveModule abstract class
    //    You should be able to hover over the methods to view the JavaDocs from there
    //----------------------------------------------------------------------------------------------


    //Module Variables
    private final int id;
    private final double offsetAngle;

    //Drive objects for the Module
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    //Steering objects for the Module
    private final TalonSRX turningMotor;
    private final CANCoder turningEncoder; 

    /**
     * @param id Arbitrary identification number (should be a label on the neo motor)
     * @param driveMotorID ID of the module's drive CANSparkMax 
     * @param turningMotorID ID of the modules turning TalonSRX motor
     * @param driveMotorID ID of the module's CANCoder for turning
     * @param offsetAngle Offset of the module in radians
     */
    public SwerveModule(int id, int driveMotorID, int turningMotorID, int turningEncoderID, double offsetAngle) {
        this.id = id;
        this.offsetAngle = offsetAngle;

        //Create the SparkMax for the drive motor, and configure the units for its encoder
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless); 
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();

        configDriveMotor();
        configDriveEncoder();
        configDrivePID();
        
        turningMotor = new TalonSRX(turningMotorID);
        turningEncoder = new CANCoder(turningEncoderID); //Our CANCoders are configured to be IDs 5-8

        configTurningEncoder();
        configTurningMotor();

        resetEncoders();
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
    }

    private void configDriveEncoder() {
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    }

    private void configDrivePID() {
        drivePID.setP(ModuleConstants.kPDrive);
        drivePID.setI(ModuleConstants.kIDrive);
        drivePID.setD(ModuleConstants.kDDrive);
        drivePID.setIZone(ModuleConstants.kIZoneDrive);
        drivePID.setFF(ModuleConstants.kFFDrive);
        drivePID.setOutputRange(-1, 1);
    }

    private void configTurningMotor() {
        //Set the CANCoder to be the sensor for the Talon's feedback loop
        turningMotor.configRemoteFeedbackFilter(turningEncoder, ModuleConstants.kPIDRemoteOrdinal);
        turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, ModuleConstants.kPIDSlotID, ModuleConstants.kPIDTimeoutMs);
        turningMotor.config_kP(ModuleConstants.kPIDSlotID, ModuleConstants.kPTurning);
        turningMotor.config_kI(ModuleConstants.kPIDSlotID, ModuleConstants.kITurning);
        turningMotor.config_kD(ModuleConstants.kPIDSlotID, ModuleConstants.kDTurning);
        turningMotor.config_kF(ModuleConstants.kPIDSlotID, ModuleConstants.kFTurning);
        turningMotor.setSelectedSensorPosition(turningEncoder.getAbsolutePosition());
    }

    private void configTurningEncoder() {
        turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turningEncoder.setPositionToAbsolute();
    }

    /**
    * @return the position of the drive motor in meters
    */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    /**
     * @return the driving velocity of the module in meters per second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * @return The absolute position of the turning motor in degrees 
     * you will probably have to convert to radians if you use this
     */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /**
     * @return The turning velocity of the module in degrees per second
     * prob never gonna use this piece of shit but I like it for continuity
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * @return The current state of the module in the form of a SwerveModuleState class: 
     * the wheel's measured velocity in meters per second and its angle in the form of a Rotation2d (in degrees)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getRotation2d());
    }

    /**
     * @return The current position of the module in the form of a SwerveModulePosition class: 
     * the wheel's measured distance in meters and its angle in the form of a Rotation2d (in degrees)
     *                ^  
     * what does this mean? No fucking clue 
     * This function is mostly just for the odometry
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }

    /**
     * @return The current angle of the module as a Rotation2d
     * Note angle is negated
     */
    public Rotation2d getRotation2d() {
        double currentPosition = turningMotor.getSelectedSensorPosition();
        double angle = encoderUnitsToRadians(currentPosition) + offsetAngle;
        return new Rotation2d(-angle);
    }

    public void setDesiredState(SwerveModuleState state) {
        //Ignore small states like when we let go of left stick so wheels don't default to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        //optimize the state, makes it so the wheel never has to travel more than 90 degrees
        state = SwerveModuleState.optimize(state, getRotation2d());

        //Drive Speed with spark and PID (or by percent output using the 2nd line)
        //drivePID.setReference(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.neoMaxRPM, CANSparkMax.ControlType.kVelocity);
        //The set function of the Neos actually take in a ratio, which is why you can't just pass in the speed of the desired state
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        //Convert [0, 360) in degrees to [0, 4906] in ticks (TalonSRX reads 4096 ticks from 360 degrees)
        //Fancy IEEE remainder algorithm to keep the angle with in 360 degrees
        //Delta has to be negated since Positive is CCW and Negative is CW for our swerve modules
        double currentPosition = turningMotor.getSelectedSensorPosition();
        double desiredPosition = radiansToEncoderUnits(state.angle.getRadians());
        double deltaPosition = Math.IEEEremainder(desiredPosition - currentPosition, ModuleConstants.kTurningEncoderTicksPerRev);
        turningMotor.set(TalonSRXControlMode.Position, currentPosition - deltaPosition);
    }

    /**
     * @param ticks The talon position you wish to convert to radians
     * @return The position in radians [0, 360]
     */
    private double encoderUnitsToRadians(double ticks) {
        return ticks * ((2 * Math.PI) / ModuleConstants.kTurningEncoderTicksPerRev);
    }

    /**
     * @param radians The angle of the talon in radians you wish to convert to talon units
     * @return The position in talon units [0, 4096]
     */
    private double radiansToEncoderUnits(double radians) {
        return radians * (ModuleConstants.kTurningEncoderTicksPerRev / (2 * Math.PI));
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(ControlMode.PercentOutput, 0);
        //drivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
    }
}