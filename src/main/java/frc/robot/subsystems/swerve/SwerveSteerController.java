package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import frc.robot.Constants.ModuleConstants;

public class SwerveSteerController {
    private final TalonSRX motor;
    private final CANCoder cancoder;

    /**
     * @param motorPort The port number of the drive motor
     * @param cancoderPort The port number of the cancoder
     * @param cancoderOffset The cancoder offset in degrees
     */
    public SwerveSteerController(int motorPort, int cancoderPort, double cancoderOffset) {
        //cancoder
        CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        cancoderConfig.magnetOffsetDegrees = cancoderOffset;
        cancoderConfig.sensorDirection = false;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        cancoder = new CANCoder(cancoderPort);
        cancoder.configAllSettings(cancoderConfig);
        cancoder.setPositionToAbsolute();

        //motor
        TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();
        motorConfig.slot0.kP = ModuleConstants.kPTurn;
        motorConfig.slot0.kI = ModuleConstants.kITurn;
        motorConfig.slot0.kD = ModuleConstants.kDTurn;
        motorConfig.slot0.kF = ModuleConstants.kFTurn;

        motor = new TalonSRX(motorPort);
        motor.configAllSettings(motorConfig);
        motor.configRemoteFeedbackFilter(cancoder, 0);
        motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 20);
    }

    /**
     * Returns the angle in degrees from 0 to 360 degrees
     * @return the position in degrees
     */
    public double getAngle() {
        double angle = motor.getSelectedSensorPosition() * ModuleConstants.kTurnPositionCoefficient;
        angle %= 360;
        angle += (angle < 0) ? 360 : 0;
        return angle;
    }

    /**
     * @param targetAngle The desired angle to set the module to in degrees
     */
    public void setAngle(double targetAngle) {
        double position = motor.getSelectedSensorPosition() + (targetAngle - getAngle()) / ModuleConstants.kTurnPositionCoefficient; 
        motor.set(ControlMode.Position, position);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Set the idle mode of the module to either Brake or Coast
     * @param brake True for brake, false for Coast
     */
    public void setBrake(boolean brake) {
        motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }
}
