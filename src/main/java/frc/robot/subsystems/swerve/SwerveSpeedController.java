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

    public SwerveSpeedController(int motorPort, ShuffleboardContainer container) {
        //motor
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.burnFlash();

        //encoder
        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        encoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MetersPerSec);
        encoder.setPosition(0);

        //pid
        pid = motor.getPIDController();
        pid.setP(ModuleConstants.kDriveP);
        pid.setI(ModuleConstants.kDriveI);
        pid.setD(ModuleConstants.kDriveD);
        pid.setIZone(ModuleConstants.kDriveIZone);
        pid.setFF(ModuleConstants.kDriveFF);
        pid.setOutputRange(-1, 1);
        
        container.addNumber("Current Position", () -> getPosition());
        container.addNumber("Current Velocity", () -> getVelocity());
    }

    /**
     * Returns the position in meters
     * @return the position in m
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
     * @param velocity The target velocity to set the 
     */
    public void setVelocity(double velocity) {
        pid.setReference(velocity / ModuleConstants.kDriveVelocityCoefficient, ControlType.kVelocity);
    }

    public void setIdleMode(IdleMode mode) {
        motor.setIdleMode(mode);
    }
}
