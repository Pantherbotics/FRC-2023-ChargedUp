package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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

    private final TalonFX telescopicMotor;

    public Arm() {
        //pivot master
        pivotLeft = new CANSparkMax(5, MotorType.kBrushless);
        pivotLeft.restoreFactoryDefaults();
        pivotLeft.setIdleMode(IdleMode.kBrake);
        pivotLeft.setSoftLimit(SoftLimitDirection.kForward, 0);
        pivotLeft.setSoftLimit(SoftLimitDirection.kReverse, 0);
        pivotLeft.enableSoftLimit(SoftLimitDirection.kForward, false);
        pivotLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        pivotLeft.burnFlash();

        //pivot slave
        pivotRight = new CANSparkMax(6, MotorType.kBrushless);
        pivotRight.restoreFactoryDefaults();
        pivotRight.setIdleMode(IdleMode.kBrake);
        pivotRight.setSoftLimit(SoftLimitDirection.kForward, 0);
        pivotRight.setSoftLimit(SoftLimitDirection.kReverse, 0);
        pivotRight.enableSoftLimit(SoftLimitDirection.kForward, false);
        pivotRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        pivotRight.burnFlash();

        pivotRight.follow(pivotLeft); //slave follows master

        //pivot encoder
        pivotEncoder = pivotLeft.getEncoder();
        pivotEncoder.setPositionConversionFactor(ArmConstants.kPivotEncoderRot2Degrees);
        pivotEncoder.setVelocityConversionFactor(ArmConstants.kPivotEncoderRPM2DegreesPerSec);

        //pivot pid
        pivotPID = pivotLeft.getPIDController();
        pivotPID.setP(0);
        pivotPID.setI(0);
        pivotPID.setD(0);
        pivotPID.setIZone(0);
        pivotPID.setFF(0);

        //cancoder = new CANCoder(0);
        
        //telescoping motor
        telescopicMotor = new TalonFX(5);
        telescopicMotor.configFactoryDefault();
        telescopicMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void pivot(boolean reversed) {
        pivotLeft.set(reversed ? -0.1 : 0.1);
        pivotRight.set(reversed ? -0.1 : 0.1);
    }

    public void stopPivot() {
        pivotLeft.set(0);
        pivotRight.set(0);
    }

    public void setIdleMode(IdleMode mode) {
        pivotLeft.setIdleMode(mode);
        pivotRight.setIdleMode(mode);
    }

    public void telescope(boolean reversed) {
        telescopicMotor.set(ControlMode.PercentOutput, reversed ? -0.3 : 0.3);
    }

    public void stopTelescope() {
        telescopicMotor.set(ControlMode.PercentOutput, 0);
    }
}
