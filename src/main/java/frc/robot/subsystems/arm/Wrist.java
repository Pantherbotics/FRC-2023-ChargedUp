package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private final CANSparkMax flexMotor, rotateMotor;
    private final RelativeEncoder flexEncoder, rotateEncoder;
    private final SparkMaxPIDController flexPID, rotatePID;

    public Wrist() {
        //flex motor
        flexMotor = new CANSparkMax(0, MotorType.kBrushless);
        flexMotor.restoreFactoryDefaults();
        flexMotor.burnFlash();

        //flex encoder
        flexEncoder = flexMotor.getEncoder();
        flexEncoder.setPositionConversionFactor(360); //360 degrees per rotation
        flexEncoder.setVelocityConversionFactor(6); //6 degrees per second

        //flex pid
        flexPID = flexMotor.getPIDController();
        flexPID.setP(0);
        flexPID.setP(0);
        flexPID.setP(0);
        flexPID.setIZone(0);
        flexPID.setFF(0);

        //rotate motor
        rotateMotor = new CANSparkMax(0, MotorType.kBrushless);

        //rotate encoder
        rotateEncoder = rotateMotor.getEncoder();

        //rotate pid
        rotatePID = rotateMotor.getPIDController();
        rotatePID.setP(0);
        rotatePID.setP(0);
        rotatePID.setP(0);
        rotatePID.setIZone(0);
        rotatePID.setFF(0);
    }

    public void flex(double speed) {
        flexPID.setReference(speed, ControlType.kVelocity);
    }

    public void rotate(double speed) {
        rotatePID.setReference(speed, ControlType.kVelocity);
    }

    public void stop() {
        flexPID.setReference(0, ControlType.kVelocity);
        rotatePID.setReference(0, ControlType.kVelocity);
    }

    @Override
    public void periodic() {

    }

    public void outputTelemetry() {

    }
}