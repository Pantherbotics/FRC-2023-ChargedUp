package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants.ModuleConstants;

public class SwerveSteerController {
    private final TalonSRX motor;
    private final CANCoder cancoder;

    public SwerveSteerController(int motorPort, int cancoderPort, double cancoderOffset, ShuffleboardContainer container) {
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
        motorConfig.slot0.kP = ModuleConstants.kTurnP;
        motorConfig.slot0.kI = ModuleConstants.kTurnI;
        motorConfig.slot0.kD = ModuleConstants.kTurnD;
        motorConfig.slot0.kF = ModuleConstants.kTurnF;

        motor = new TalonSRX(motorPort);
        motor.configAllSettings(motorConfig);
        motor.configRemoteFeedbackFilter(cancoder, 0);
        motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 20);

        container.addNumber("Current Angle", () -> getAngle());
    }

    /**
     * Returns the angle in degrees [0, 360]
     * @return the position in degrees
     */
    public double getAngle() {
        double angle = motor.getSelectedSensorPosition() * ModuleConstants.kTurnPositionCoefficient;
        angle %= 360;
        angle += (angle < 0) ? 360 : 0;
        return angle;
    }

    public void setAngle(double angle) {
        motor.set(ControlMode.Position, angle);
    }

    public void setNeutralMode(NeutralMode mode) {
        motor.setNeutralMode(mode);
    }
}
