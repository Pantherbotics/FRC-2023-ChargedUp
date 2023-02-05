package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final CANSparkMax pivotMaster, pivotFollower;
    private final CANCoder pivotEncoder; 
    private final PIDController pivotPID;

    private final TalonFX extensionMotor;

    public Arm() {
        pivotMaster = new CANSparkMax(0, MotorType.kBrushless);
        pivotMaster.restoreFactoryDefaults();
        pivotMaster.setIdleMode(IdleMode.kCoast);
        pivotMaster.enableSoftLimit(SoftLimitDirection.kForward, true);
        pivotMaster.setSoftLimit(SoftLimitDirection.kForward, 0);
        pivotMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        pivotMaster.setSoftLimit(SoftLimitDirection.kReverse, 0);
        pivotMaster.burnFlash();

        pivotFollower = new CANSparkMax(0, MotorType.kBrushless);
        pivotFollower.restoreFactoryDefaults();
        pivotFollower.setIdleMode(IdleMode.kCoast);
        pivotFollower.burnFlash();
        pivotFollower.follow(pivotMaster, true);

        pivotEncoder = new CANCoder(0);

        pivotPID = new PIDController(0, 0, 0);

        extensionMotor = new TalonFX(0);
    }

    public void pivot(double speed) {
        pivotMaster.set(speed);
    }

    public void extend() {
        extensionMotor.set(ControlMode.Position, 300);
    }

    public void extendOpenLoop() {

    }

    public void stopExtension() {
        extensionMotor.set(ControlMode.PercentOutput, 0);
    }

    public void stopPivot() {
        pivotMaster.set(0);
    }
}
