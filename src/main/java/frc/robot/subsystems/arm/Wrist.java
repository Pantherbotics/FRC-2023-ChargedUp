package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
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

public class Wrist extends SubsystemBase implements PIDTuner{
    private final CANSparkMax flexMotor, rotateMotor;
    private final RelativeEncoder flexEncoder, rotateEncoder;
    private final SparkMaxPIDController flexPID, rotatePID;
    private double localP, localI, localD;

    public Wrist() {
        // flex motor
        flexMotor = new CANSparkMax(8, MotorType.kBrushless);
        flexMotor.restoreFactoryDefaults();
        flexMotor.setIdleMode(IdleMode.kCoast);
        flexMotor.setSoftLimit(SoftLimitDirection.kForward, 50);
        flexMotor.setSoftLimit(SoftLimitDirection.kReverse, -50);
        flexMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        flexMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        flexMotor.burnFlash();

        // flex encoder
        flexEncoder = flexMotor.getEncoder();
        flexEncoder.setPositionConversionFactor(360); // 360 degrees per rotation
        flexEncoder.setVelocityConversionFactor(6); // 6 degrees per second

        // flex pid
        flexPID = flexMotor.getPIDController();
        flexPID.setP(ArmConstants.kFlexP);
        flexPID.setI(ArmConstants.kFlexI);
        flexPID.setD(ArmConstants.kFlexD);
        flexPID.setIZone(ArmConstants.kFlexIZone);
        flexPID.setFF(ArmConstants.kFlexFF);

        localP = ArmConstants.kFlexP;
        localI = ArmConstants.kFlexI;
        localD = ArmConstants.kFlexD;

        SmartDashboard.putNumber("Flex kP", flexPID.getP());
        SmartDashboard.putNumber("Flex kI", flexPID.getI());
        SmartDashboard.putNumber("Flex kD", flexPID.getD());
        SmartDashboard.putNumber("Flex kIZone", flexPID.getIZone());
        SmartDashboard.putNumber("Flex kFF", flexPID.getFF());

        SmartDashboard.putNumber("Flex encoder", getFlexPosition());
        // rotate motor
        rotateMotor = new CANSparkMax(7, MotorType.kBrushless);
        rotateMotor.restoreFactoryDefaults();
        rotateMotor.setIdleMode(IdleMode.kCoast);
        rotateMotor.burnFlash();

        // rotate encoder
        rotateEncoder = rotateMotor.getEncoder();
        rotateEncoder.setPositionConversionFactor(360);
        rotateEncoder.setVelocityConversionFactor(6);

        // rotate pid
        rotatePID = rotateMotor.getPIDController();
        rotatePID.setP(ArmConstants.kFlexP);
        rotatePID.setI(ArmConstants.kFlexI);
        rotatePID.setD(ArmConstants.kFlexD);
        rotatePID.setIZone(ArmConstants.kFlexIZone);
        rotatePID.setFF(ArmConstants.kFlexFF);
        SmartDashboard.putNumber("Rotate encoder", getRotatePosition());

        // SmartDashboard.putNumber("Rotate kP", rotatePID.getP());
        // SmartDashboard.putNumber("Rotate kI", rotatePID.getI());
        // SmartDashboard.putNumber("Rotate kD", rotatePID.getD());
        // SmartDashboard.putNumber("Rotate kIZone", rotatePID.getIZone());
        // SmartDashboard.putNumber("Rotate kF", rotatePID.getFF());
    }

    public void flex(double speed) {
        // flexPID.setReference(getFlexPosition() + speed, ControlType.kVelocity);
        flexMotor.set(speed);
    }

    public double getFlexPosition() {
        return flexEncoder.getPosition();
    }

    public void rotate(double speed) {
        // rotatePID.setReference(getRotatePosition() + speed, ControlType.kVelocity);
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
        //flexPID.setP(SmartDashboard.getNumber("Flex kP", flexPID.getP()));
        //flexPID.setI(SmartDashboard.getNumber("Flex kI", flexPID.getI()));
        //flexPID.setD(SmartDashboard.getNumber("Flex kD", flexPID.getD()));
        //flexPID.setIZone(SmartDashboard.getNumber("Flex kIZone",
        //flexPID.getIZone()));
        //flexPID.setFF(SmartDashboard.getNumber("Flex kFF", flexPID.getFF()));

        // rotatePID.setP(SmartDashboard.getNumber("Rotate kP", rotatePID.getP()));
        // rotatePID.setI(SmartDashboard.getNumber("Rotate kI", rotatePID.getI()));
        // rotatePID.setD(SmartDashboard.getNumber("Rotate kD", rotatePID.getD()));
        // rotatePID.setIZone(SmartDashboard.getNumber("Rotate kIZone",
        // rotatePID.getIZone()));
        // rotatePID.setFF(SmartDashboard.getNumber("Rotate kF", rotatePID.getFF()));
    }

    @Override
    public void alterP(double val) {
        localP += val;
        flexPID.setP(localP);
    }

    @Override
    public void alterI(double val) {
        localI += val;
        flexPID.setI(localI);
    }

    @Override
    public void alterD(double val) {
        localD += val;
        flexPID.setD(localD);
    }

    @Override
    public String getIdentifier() {
        return "Wrist PID Tuner";
    }
}