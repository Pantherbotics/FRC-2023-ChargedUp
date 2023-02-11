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
    private final CANSparkMax pivotMaster, pivotSlave;
    private final RelativeEncoder pivotEncoder;
    private final SparkMaxPIDController pivotPID;

    //cancoder for the pivot
    //private final CANCoder cancoder; 

    private final TalonFX extensionMotor;

    public Arm() {
        //pivot master
        pivotMaster = new CANSparkMax(5, MotorType.kBrushless);
        pivotMaster.restoreFactoryDefaults();
        pivotMaster.setIdleMode(IdleMode.kBrake);
        pivotMaster.setSoftLimit(SoftLimitDirection.kForward, 0);
        pivotMaster.setSoftLimit(SoftLimitDirection.kReverse, 0);
        pivotMaster.enableSoftLimit(SoftLimitDirection.kForward, false);
        pivotMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        pivotMaster.burnFlash();

        //pivot slave
        pivotSlave = new CANSparkMax(6, MotorType.kBrushless);
        pivotSlave.restoreFactoryDefaults();
        pivotSlave.setIdleMode(IdleMode.kBrake);
        pivotSlave.setSoftLimit(SoftLimitDirection.kForward, 0);
        pivotSlave.setSoftLimit(SoftLimitDirection.kReverse, 0);
        pivotSlave.enableSoftLimit(SoftLimitDirection.kForward, false);
        pivotSlave.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        pivotSlave.burnFlash();

        pivotSlave.follow(pivotMaster); //slave follows master
        

        //pivot encoder
        pivotEncoder = pivotMaster.getEncoder();
        pivotEncoder.setPositionConversionFactor(ArmConstants.kPivotEncoderRot2Degrees);
        pivotEncoder.setVelocityConversionFactor(ArmConstants.kPivotEncoderRPM2DegreesPerSec);

        //pivot pid
        pivotPID = pivotMaster.getPIDController();
        pivotPID.setP(0);
        pivotPID.setI(0);
        pivotPID.setD(0);
        pivotPID.setIZone(0);
        pivotPID.setFF(0);
        pivotPID.setPositionPIDWrappingMinInput(0);
        pivotPID.setPositionPIDWrappingMaxInput(0);
        pivotPID.setPositionPIDWrappingEnabled(true);

        //cancoder = new CANCoder(0);
        
        //telescoping motor
        extensionMotor = new TalonFX(5);
    }

    public void pivot(boolean reversed) {
        double value = reversed ? -1 : 1;
        pivotMaster.set(value);
        pivotSlave.set(value);
    }

    public void stopPivot() {
        pivotMaster.set(0);
        pivotSlave.set(0);
    }

    public void extend(boolean reversed) {
        extensionMotor.set(ControlMode.PercentOutput, reversed ? -0.3 : 0.3);
    }

    public void stopExtension() {
        extensionMotor.set(ControlMode.PercentOutput, 0);
    }
}
