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
    private final int id;
    
    //Drive objects for the Module
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    //Steering objects for the Module
    private final TalonSRX turningMotor;
    private final CANCoder turningEncoder; 

    private final Rotation2d turningEncoderZero;
    private Rotation2d turningMotorOffset;

    private final double kDriveVelocityCoefficient = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / Constants.neoMaxRPM;
    private final double kTurningPositionCoefficient = 2.0 * Math.PI / 2048.0;

    /**
     * @param id Arbitrary identification number (should be a label on the neo motor)
     * @param driveMotorID ID of the module's drive CANSparkMax 
     * @param turningMotorID ID of the modules turning TalonSRX motor
     * @param driveMotorID ID of the module's CANCoder for turning
     * @param offsetAngle Offset of the module in radians
     */
    public SwerveModule(int id, int driveMotorID, int turningMotorID, int turningEncoderID, Rotation2d offsetAngle) {
        this.id = id;
        turningEncoderZero = offsetAngle;

        //drive 
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();
        
        //turning
        turningMotor = new TalonSRX(turningMotorID);
        turningEncoder = new CANCoder(turningEncoderID); 

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

        //drive pid
        drivePID.setP(0.0001);
        drivePID.setI(0.0);
        drivePID.setD(0.0001);
        drivePID.setIZone(0.0);
        drivePID.setFF(0.000175);
        drivePID.setOutputRange(-1, 1);

        //turning motor 
        turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 20);
        turningMotor.setNeutralMode(NeutralMode.Coast);
        turningMotor.setInverted(false); //counter-clockwise I think? Needs testing
        turningMotor.setSensorPhase(false);

        //turning pid configs
        turningMotor.config_kP(0, 1.0);
        turningMotor.config_kI(0, 0.0005);
        turningMotor.config_kD(0, 0.0);
        turningMotor.config_kF(0, 0.0);
        turningMotor.setSelectedSensorPosition(turningEncoder.getAbsolutePosition());

        //cancoder
        turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turningEncoder.configSensorDirection(false); //counter-clockwise
        turningEncoder.setPositionToAbsolute();

        //set the cancoder to be the remote feedback sensor for the turning motor
        turningMotor.configRemoteFeedbackFilter(turningEncoder, 0);
    }

    /**
     * @return The current state of the module in the form of a SwerveModuleState class: 
     * the wheel's measured velocity in meters per second and its angle in the form of a Rotation2d
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurningAngle());
    }

    /**
     * @return The current position of the module in the form of a SwerveModulePosition class: 
     * the wheel's measured distance in meters and its angle in the form of a Rotation2d
     *                
     * This function is mostly just for the odometry
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurningAngle());
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
     * Putting an angle in a Rotation2d actually bounds the angle because of how tangent works 
     * @return The angle of the turning motor as a Rotation2d
     */
    public Rotation2d getTurningAngle() {
        return Rotation2d.fromRadians(getUnboundTurningAngleRadians());
    }

    /**
     * @return The unbound angle of the turning motor in radians
     */
    public double getUnboundTurningAngleRadians() {
        return (turningMotor.getSelectedSensorPosition() * kTurningPositionCoefficient) - turningMotorOffset.getRadians();
    }

    public void rezeroTurningMotor() {
        Rotation2d currentAngle = Rotation2d.fromRadians(turningMotor.getSelectedSensorPosition() * kTurningPositionCoefficient);
        turningMotorOffset = currentAngle.rotateBy(getAdjustedTurningEncoderAngle().unaryMinus());
    }
    /**
     * @return The raw angle of the module's cancoder as a Rotation2d
     */
    public Rotation2d getTurningEncoderAngle() {
        return Rotation2d.fromDegrees(turningEncoder.getAbsolutePosition());
    }

    /**
     * @return The angle of module's cancoder taking into account the cancoder's offset
     */
    public Rotation2d getAdjustedTurningEncoderAngle() {
        return getTurningEncoderAngle().rotateBy(turningEncoderZero.unaryMinus());
    }

    public void setDesiredState(SwerveModuleState state) {
        //Ignore small states like when we let go of left stick so wheels don't default to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        boolean flip = setTurningAngleShortestPath(state.angle);
        drivePID.setReference(state.speedMetersPerSecond / kDriveVelocityCoefficient * (flip ? -1 : 1), ControlType.kVelocity);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //turning encoder is absolute, no need to reset 
    }

    /**
     * @param angle The desired angle in radians
     */
    private void setTurningAngleUnbound(double angle) {
        turningMotor.set(TalonSRXControlMode.Position, (angle + turningMotorOffset.getRadians()) / kTurningPositionCoefficient); 
    }
    
    /**
     * Takes the shortest path to the desired angle, similar to WPILib's optimize function
     * Should always travel 90 degrees or less 
     * @param angle The desired angle to rotate to
     * @return True if the drive velocity should be inverted, false otherwise
     */
    private boolean setTurningAngleShortestPath(Rotation2d angle) {
        boolean flip = false;
        double unboundPosition = getUnboundTurningAngleRadians();
        Rotation2d boundPosition = Rotation2d.fromRadians(unboundPosition);
        Rotation2d relativeRotation = angle.rotateBy(boundPosition.unaryMinus());
        double relativeAngleRad = relativeRotation.getRadians();

        // Flipping drive direction would be the shorter path.
        if(relativeAngleRad > Math.PI / 2.0) {
            flip = true;
            relativeAngleRad -= Math.PI;
        } else if(relativeAngleRad < -(Math.PI / 2.0)) {
            flip = true;
            relativeAngleRad += Math.PI;
        }

        setTurningAngleUnbound(unboundPosition + relativeAngleRad);
        return flip;
    }
    
    /**
     * stops the modules
     */
    public void stop() {
        drivePID.setReference(0, ControlType.kVelocity);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Swerve[" + id + "]", "Poggers");
    }
}