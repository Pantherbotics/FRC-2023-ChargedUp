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
    private final double offsetDeg;

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
     * @param offsetDeg Offset of the module in degrees
     */
    public SwerveModule(int id, int driveMotorID, int turningMotorID, int turningEncoderID, int offsetDeg) {
        this.id = id;
        this.offsetDeg = offsetDeg;

        //Create the SparkMax for the drive motor, and configure the units for its encoder
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless); 
        driveMotor.restoreFactoryDefaults();
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        //Get the Drive PID Controller and configure it for Velocity PID
        drivePID = driveMotor.getPIDController();
        drivePID.setP(ModuleConstants.kPDrive);
        drivePID.setI(ModuleConstants.kIDrive);
        drivePID.setD(ModuleConstants.kDDrive);
        drivePID.setIZone(ModuleConstants.kIZoneDrive);
        drivePID.setFF(ModuleConstants.kFFDrive);
        drivePID.setOutputRange(-1, 1);

        //Create the Steer TalonSRX
        turningMotor = new TalonSRX(turningMotorID); //set in id ())create talon object

        //Create the CANCoder and configure it to work as the RemoteSensor0 for the steer motor
        turningEncoder = new CANCoder(turningEncoderID); //Our CANCoders are configured to be IDs 5-8
        turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turningEncoder.setPositionToAbsolute();
        turningMotor.configRemoteFeedbackFilter(turningEncoder, 0);
        turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 20);
        turningMotor.config_kP(0, ModuleConstants.kPTurning);
        turningMotor.config_kI(0, ModuleConstants.kITurning);
        turningMotor.config_kD(0, ModuleConstants.kDTurning);
        turningMotor.config_kF(0, ModuleConstants.kFTurning);
        turningMotor.setSelectedSensorPosition(turningEncoder.getAbsolutePosition());

        //Reset the encoders
        resetEncoders();
    }

    /**
    * @return the position of the drive motor
    */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    /**
     * @return the driving velocity of the module
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * @return The position of the turning motor (absolute not relative)
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
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * @return The current position of the module in the form of a SwerveModulePosition class: 
     * the wheel's measured distance in meters and its angle in the form of a Rotation2d (in degrees)
     *                ^  
     * what does this mean? No fucking clue 
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getAngle() {
        double angle = turningMotor.getSelectedSensorPosition() * 360.0 / 4096 + offsetDeg;
        while (angle > 360) { angle -= 360; }
		while (angle < 0) { angle += 360; }
		return angle - 180;
    }

    public double getAbsoluteEncoderRad() {
        return Math.toRadians(getAngle());
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public void setDesiredState(SwerveModuleState state) {
        //Ignore small states like when we let go of left stick so wheels don't default to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        //optimize the state, makes it so the wheel never has to travel more than 90 degrees
        state = SwerveModuleState.optimize(state, state.angle);
        double target = state.angle.getDegrees();
        //Log some info about the target state
        SmartDashboard.putString("Swerve[" + id + "] Set", "Angle: " + target + " Speed: " + state.speedMetersPerSecond);

        //Convert [0, 360) in degrees to [0, 4906] in ticks (TalonSRX reads 4096 ticks from 360 degrees)
        //Error has to be negated since Positive is CCW and Negative is CW for our swerve modules       
        double position = turningMotor.getSelectedSensorPosition() + (-1) * (target - getAngle()) * (4096.0 / 360);
        turningMotor.set(TalonSRXControlMode.Position, position);

        //Drive Speed with spark and PID (or by percent output using the 2nd line)
        drivePID.setReference(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.neoMaxRPM, CANSparkMax.ControlType.kVelocity);
        //driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }

    public void stop() {
        //driveMotor.set(0);
        drivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}