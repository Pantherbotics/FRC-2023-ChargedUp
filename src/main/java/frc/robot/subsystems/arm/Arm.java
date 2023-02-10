package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax pivotLeft, pivotRight;
    private final RelativeEncoder pivotEncoder;
    private final SparkMaxPIDController pivotPID;

    //cancoder for the pivot
    //private final CANCoder cancoder; 

    private final TalonFX extensionMotor;

    public Arm() {
        //pivot motors, they basically do the exact same thing
        pivotLeft = new CANSparkMax(5, MotorType.kBrushless);
        pivotRight = new CANSparkMax(6, MotorType.kBrushless);
        
        pivotEncoder = pivotLeft.getEncoder();

        pivotPID = pivotLeft.getPIDController();

        //cancoder = new CANCoder(0);
        
        //telescoping motor
        extensionMotor = new TalonFX(5);

        configDevices();
    }

    private void configDevices() {
        pivotLeft.restoreFactoryDefaults();
        pivotLeft.setIdleMode(IdleMode.kCoast);
        pivotLeft.setInverted(false);
        pivotLeft.setSoftLimit(SoftLimitDirection.kForward, 0);
        pivotLeft.setSoftLimit(SoftLimitDirection.kReverse, 0);
        pivotLeft.enableSoftLimit(SoftLimitDirection.kForward, false);
        pivotLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        pivotLeft.burnFlash();

        pivotRight.setIdleMode(IdleMode.kCoast);
        pivotRight.setInverted(true);
        pivotRight.setSoftLimit(SoftLimitDirection.kForward, 0);
        pivotRight.setSoftLimit(SoftLimitDirection.kReverse, 0);
        pivotRight.enableSoftLimit(SoftLimitDirection.kForward, false);
        pivotRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        pivotRight.burnFlash();

        //pivot encoder, only need one
        pivotEncoder.setPositionConversionFactor(ArmConstants.kPivotEncoderRot2Degrees);
        pivotEncoder.setVelocityConversionFactor(ArmConstants.kPivotEncoderRPM2DegreesPerSec);

        /*pivot pid, again only need one
        pivotPID.setP(0);
        pivotPID.setI(0);
        pivotPID.setD(0);
        pivotPID.setIZone(0);
        pivotPID.setFF(0);
        pivotPID.setPositionPIDWrappingMinInput(0);
        pivotPID.setPositionPIDWrappingMaxInput(0);
        pivotPID.setPositionPIDWrappingEnabled(true);*/
    }

    public void pivot(boolean reversed) {
        double value = reversed ? -1 : 1;
        pivotLeft.set(value);
        pivotRight.set(value);
    }

    public void stopPivot() {
        pivotLeft.set(0);
        pivotRight.set(0);
    }

    public void extend(boolean reversed) {
        extensionMotor.set(ControlMode.PercentOutput, reversed ? -0.3 : 0.3);
    }

    public void stopExtension() {
        extensionMotor.set(ControlMode.PercentOutput, 0);
    }
}
