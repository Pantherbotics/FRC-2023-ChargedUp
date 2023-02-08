package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    //----------------------------------------------------------------------------------------------
    //  All of the following code is documented in the SwerveModule abstract class
    //    You should be able to hover over the methods to view the JavaDocs from there
    //----------------------------------------------------------------------------------------------


    //Module Variables
    private final String moduleName;
    
    //Drive objects for the Module
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    //Steering objects for the Module
    private final TalonSRX turnMotor;
    private final CANCoder cancoder; 
    private final double moduleOffset;

    private final double kDriveVelocityCoefficient = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / Constants.neoMaxRPM;
    private final double kTurnPositionCoefficient = 360.0 / 4069.0;

    /**
     * @param moduleNumber Arbitrary identification number (should be a label on the neo motor)
     * @param driveMotorID ID of the module's drive CANSparkMax 
     * @param turnMotorID ID of the modules turning TalonSRX motor
     * @param driveMotorID ID of the module's CANCoder for turning
     * @param angleOffset Offset of the module in degrees
     */
    public SwerveModule(int moduleNumber, int driveMotorID, int turnMotorID, int turnEncoderID, int angleOffset) {
        moduleName = "Swerve[" + moduleNumber + "]";

        //drive 
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();
        
        //turning
        turnMotor = new TalonSRX(turnMotorID);
        cancoder = new CANCoder(turnEncoderID); 
        moduleOffset = angleOffset;

        configDevices();

        resetEncoders();
    }

    private void configDevices() {
        //drive motor
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setSmartCurrentLimit(40);
        driveMotor.burnFlash();

        //drive encoder
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveEncoder.setPosition(0);

        //drive pid
        drivePID.setP(0.0001);
        drivePID.setI(0.0);
        drivePID.setD(0.0001);
        drivePID.setIZone(0.0);
        drivePID.setFF(0.000175);
        drivePID.setOutputRange(-1, 1);

        //turning motor 
        turnMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20); //setup to use the cancoder as an encoder
        turnMotor.setNeutralMode(NeutralMode.Coast);
        turnMotor.setInverted(false); //counter-clockwise I think? Needs testing
        turnMotor.setSensorPhase(false);

        //turning pid configs
        turnMotor.config_kP(0, 1.0);
        turnMotor.config_kI(0, 0.0005);
        turnMotor.config_kD(0, 0.0);
        turnMotor.config_kF(0, 0.0);
        turnMotor.setSelectedSensorPosition(cancoder.getAbsolutePosition());

        //cancoder
        cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        cancoder.configSensorDirection(false); //counter-clockwise
        cancoder.setPositionToAbsolute();

        //set the cancoder to be the remote feedback sensor for the turning motor
        //turningMotor.configRemoteFeedbackFilter(turningEncoder, 0);
    }

    public void setDesiredState(SwerveModuleState state) {
        //Ignore small states like when we let go of left stick so wheels don't default to 0 degrees
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getTurnAngle()));
        turnMotor.set(TalonSRXControlMode.Position, state.angle.getRadians() / kTurnPositionCoefficient);
        drivePID.setReference(state.speedMetersPerSecond / kDriveVelocityCoefficient, ControlType.kVelocity);
    }

    /**
     * @return The current state of the module in the form of a SwerveModuleState class: 
     * the wheel's measured velocity in meters per second and its angle in the form of a Rotation2d
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurnAngle()));
    }

    /**
     * @return The current position of the module in the form of a SwerveModulePosition class: 
     * the wheel's measured distance in meters and its angle in the form of a Rotation2d
     *                
     * This function is mostly just for the odometry
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurnAngle()));
    }
    
    /**
    * @return The position of the drive motor in meters
    */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    /**
     * @return The driving velocity of the module in meters per second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * @return The angle of the turning motor in degrees from 0 to 360
     */
    public double getTurnAngle() {
        return (turnMotor.getSelectedSensorPosition() * kTurnPositionCoefficient) % 360;
    }

    /**
     * @return The raw angle of the module's cancoder as a Rotation2d
     */
    public double getCancoderAngle() {
        return cancoder.getAbsolutePosition() - moduleOffset;
    }
    
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //turning encoder is absolute, no need to reset 
    }
    
    /**
     * stops the modules
     */
    public void stop() {
        drivePID.setReference(0, ControlType.kVelocity);
        turnMotor.set(ControlMode.PercentOutput, 0);
    }

    //output module data to SmartDashboard
    public void outputTelemetry() {
        SmartDashboard.putNumber(moduleName + " cancoder", cancoder.getPosition());
        SmartDashboard.putNumber(moduleName + " cancoder w/ offset", getCancoderAngle());
        SmartDashboard.putNumber(moduleName + " error", turnMotor.getClosedLoopError());
        SmartDashboard.putNumber(moduleName + " error degrees", turnMotor.getClosedLoopError() * kTurnPositionCoefficient);
        SmartDashboard.putNumber(moduleName + " turn motor", turnMotor.getSelectedSensorPosition() * kTurnPositionCoefficient);
    }
}