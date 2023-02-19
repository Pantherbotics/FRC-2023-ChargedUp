package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PIDTuner;
import frc.robot.Constants.ArmConstants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax flexMotor, rotateMotor;
    private final RelativeEncoder flexEncoder, rotateEncoder;
    private final SparkMaxPIDController flexPID, rotatePID;

    public boolean isOpenLoop = true;

    public Wrist() {
        // flex motor
        flexMotor = new CANSparkMax(8, MotorType.kBrushless);
        flexMotor.restoreFactoryDefaults();
        flexMotor.setIdleMode(IdleMode.kCoast);
        flexMotor.setSoftLimit(SoftLimitDirection.kForward, -1);
        flexMotor.setSoftLimit(SoftLimitDirection.kReverse, 260);
        flexMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        flexMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // flex encoder
        flexEncoder = flexMotor.getEncoder();
        flexEncoder.setPositionConversionFactor(360); // 360 degrees per rotation
        flexEncoder.setVelocityConversionFactor(6); // 6 degrees per second
        flexEncoder.setPosition(0);

        // flex pid
        flexPID = flexMotor.getPIDController();
        flexPID.setP(ArmConstants.kPFlex);
        flexPID.setI(ArmConstants.kIFlex);
        flexPID.setD(ArmConstants.kDFlex);
        flexPID.setIZone(ArmConstants.kIZoneFlex);
        flexPID.setFF(ArmConstants.kFFFlex);
        flexPID.setOutputRange(-1, 1);

        flexMotor.burnFlash();

        // rotate motor
        rotateMotor = new CANSparkMax(7, MotorType.kBrushless);
        rotateMotor.restoreFactoryDefaults();
        rotateMotor.setIdleMode(IdleMode.kCoast);

        // rotate encoder
        rotateEncoder = rotateMotor.getEncoder();
        rotateEncoder.setPositionConversionFactor(360);
        rotateEncoder.setVelocityConversionFactor(6);

        // rotate pid
        rotatePID = rotateMotor.getPIDController();
        rotatePID.setP(ArmConstants.kPRotate);
        rotatePID.setI(ArmConstants.kIRotate);
        rotatePID.setD(ArmConstants.kDRotate);
        rotatePID.setIZone(ArmConstants.kIZoneRotate);
        rotatePID.setFF(ArmConstants.kFFRotate);
        rotatePID.setOutputRange(-1, 1);

        rotateMotor.burnFlash();

        // SmartDashboard.putNumber("Flex kP", flexPID.getP());
        // SmartDashboard.putNumber("Flex kI", flexPID.getI());
        // SmartDashboard.putNumber("Flex kD", flexPID.getD());
        // SmartDashboard.putNumber("Flex kIZone", flexPID.getIZone());
        // SmartDashboard.putNumber("Flex kFF", flexPID.getFF());

        // SmartDashboard.putNumber("Rotate kP", rotatePID.getP());
        // SmartDashboard.putNumber("Rotate kI", rotatePID.getI());
        // SmartDashboard.putNumber("Rotate kD", rotatePID.getD());
        // SmartDashboard.putNumber("Rotate kIZone", rotatePID.getIZone());
        // SmartDashboard.putNumber("Rotate kFF", rotatePID.getFF());
    }

    public void flexClosedLoop(double speed) {
        speed *= 2;
        SmartDashboard.putNumber("Flex Speed", speed);
        flexPID.setReference(speed, ControlType.kVoltage);
    }

    public void flexOpenLoop(double speed) {
        flexMotor.set(speed);
    }

    public double getFlexPosition() {
        return flexEncoder.getPosition();
    }

    public void rotateClosedLoop(double speed) {
        speed = speed * 5700;
        SmartDashboard.putNumber("Rotate Speed", speed);
        rotatePID.setReference(speed, ControlType.kVelocity);
    }

    public void rotateOpenLoop(double speed) {
        rotateMotor.set(speed);
    }

    public double getRotatePosition() {
        return rotateEncoder.getPosition();
    }

    public void stop() {
        flexPID.setReference(0, ControlType.kVelocity);
        rotatePID.setReference(0, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flex kP", flexPID.getP());
        SmartDashboard.putNumber("Flex kI", flexPID.getI());
        SmartDashboard.putNumber("Flex kD", flexPID.getD());
        SmartDashboard.putNumber("Flex kIZone", flexPID.getIZone());
        SmartDashboard.putNumber("Flex kFF", flexPID.getFF());

        SmartDashboard.putNumber("Rotate kP", rotatePID.getP());
        SmartDashboard.putNumber("Rotate kI", rotatePID.getI());
        SmartDashboard.putNumber("Rotate kD", rotatePID.getD());
        SmartDashboard.putNumber("Rotate kIZone", rotatePID.getIZone());
        SmartDashboard.putNumber("Rotate kFF", rotatePID.getFF());

        // flexPID.setP(SmartDashboard.getNumber("Flex kP", flexPID.getP()));
        // flexPID.setI(SmartDashboard.getNumber("Flex kI", flexPID.getI()));
        // flexPID.setD(SmartDashboard.getNumber("Flex kD", flexPID.getD()));
        // flexPID.setIZone(SmartDashboard.getNumber("Flex IZone", flexPID.getIZone()));
        // flexPID.setFF(SmartDashboard.getNumber("Flex kFF", flexPID.getFF()));

        // rotatePID.setP(SmartDashboard.getNumber("Rotate kP", rotatePID.getP()));
        // rotatePID.setI(SmartDashboard.getNumber("Rotate kI", rotatePID.getI()));
        // rotatePID.setD(SmartDashboard.getNumber("Rotate kD", rotatePID.getD()));
        // rotatePID.setIZone(SmartDashboard.getNumber("Rotate IZone", rotatePID.getIZone()));
        // rotatePID.setFF(SmartDashboard.getNumber("Rotate kFF", rotatePID.getFF()));

        SmartDashboard.putBoolean("Open Loop", isOpenLoop);

        SmartDashboard.putNumber("Flex Position", getFlexPosition());
        SmartDashboard.putNumber("Rotate Position", getRotatePosition());
    }
}