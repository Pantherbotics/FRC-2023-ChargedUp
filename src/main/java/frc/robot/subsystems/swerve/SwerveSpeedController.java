package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants.ModuleConstants;

public class SwerveSpeedController {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;

    /**
     * 
     * @param motorPort The port number of the turn motor
     * @param container
     */
    public SwerveSpeedController(int motorPort, ShuffleboardContainer container) {
        //motor
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        //encoder
        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        encoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MetersPerSec);
        encoder.setPosition(0);

        //pid
        pid = motor.getPIDController();
        pid.setP(ModuleConstants.kPDrive);
        pid.setI(ModuleConstants.kIDrive);
        pid.setD(ModuleConstants.kDDrive);
        pid.setIZone(ModuleConstants.kIZoneDrive);
        pid.setFF(ModuleConstants.kFFDrive);
        pid.setOutputRange(-1, 1);

        motor.burnFlash();
        
        //container.addNumber("Current Swerve Position", () -> getPosition());
        //container.addNumber("Current Swerve Velocity", () -> getVelocity());
    }

    /**
     * Returns the position in meters
     * @return the position in meters
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Returns the velocity in meters per second
     * @return the velocity in m/s
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Sets the drive motor the desired velocity in meters per second
     * @param targetVelocity The target velocity to set the module to in meters per second
     */
    public void setVelocity(double targetVelocity) {
        pid.setReference(targetVelocity / ModuleConstants.kDriveVelocityCoefficient, ControlType.kVelocity);
    }

    public void stop() {
        motor.stopMotor();
    }

    /**
     * Set the idle mode of the module to either Brake or Coast
     * @param brake True for brake, false for Coast
     */
    public void setBrake(boolean brake) {
        motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
