package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final CANSparkMax leftPivotMotor, rightPivotMotor;
    private final CANCoder pivotEncoder; 

    private final TalonFX extensionMotor;
    
    public Arm() {
        leftPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(0, MotorType.kBrushless);

        pivotEncoder = new CANCoder(0);

        extensionMotor = new TalonFX(0);
    }
}
