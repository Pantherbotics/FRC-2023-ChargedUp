package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax pivotLeader, pivotFollower;
    private final RelativeEncoder pivotEncoder;
    private final SparkMaxPIDController pivotPID;

    // cancoder for the pivot
    private final CANCoder cancoder;

    private double pivotSetpoint;

    // extension
    private final TalonFX extensionMotor;

    private double extensionSetpoint;

    public Arm() {
        // pivot leader
        pivotLeader = new CANSparkMax(ArmConstants.kPivotLeaderMotorPort, MotorType.kBrushless);
        pivotLeader.restoreFactoryDefaults();
        pivotLeader.setIdleMode(IdleMode.kBrake);
        pivotLeader.setSoftLimit(SoftLimitDirection.kForward, 0);
        pivotLeader.setSoftLimit(SoftLimitDirection.kReverse, 0);
        pivotLeader.enableSoftLimit(SoftLimitDirection.kForward, false);
        pivotLeader.enableSoftLimit(SoftLimitDirection.kReverse, false);

        // pivot follower
        pivotFollower = new CANSparkMax(ArmConstants.kPivotFollowerMotorPort, MotorType.kBrushless);
        pivotFollower.restoreFactoryDefaults();
        pivotFollower.setIdleMode(IdleMode.kBrake);
        pivotFollower.follow(pivotLeader);

        // pivot encoder
        pivotEncoder = pivotLeader.getEncoder();
        pivotEncoder.setPositionConversionFactor(ArmConstants.kPivotEncoderRot2Degrees);
        pivotEncoder.setVelocityConversionFactor(ArmConstants.kPivotEncoderRPM2DegreesPerSec);
        pivotEncoder.setPosition(0);

        // pivot pid
        pivotPID = pivotLeader.getPIDController();
        pivotPID.setP(0.4);
        pivotPID.setI(0);
        pivotPID.setD(0);
        pivotPID.setIZone(0);
        pivotPID.setFF(0);

        pivotLeader.burnFlash();
        pivotFollower.burnFlash();

        pivotSetpoint = getPivotAngle();

        // cancoder
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = 0; // change

        cancoder = new CANCoder(0);
        cancoder.configAllSettings(config);

        // extension motor
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.slot0.kP = 0.3;
        motorConfig.slot0.kI = 0;
        motorConfig.slot0.kD = 0;
        motorConfig.slot0.kF = 0;
        motorConfig.forwardSoftLimitThreshold = 1000000;
        motorConfig.reverseSoftLimitThreshold = 0;
        motorConfig.forwardSoftLimitEnable = true;
        motorConfig.reverseSoftLimitEnable = true;

        extensionMotor = new TalonFX(5);
        extensionMotor.configAllSettings(motorConfig);
        extensionMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        extensionMotor.setSelectedSensorPosition(0); // 0 position should be when the arm is fully down
        extensionMotor.setNeutralMode(NeutralMode.Brake);
        extensionMotor.setInverted(TalonFXInvertType.CounterClockwise);
        
        extensionSetpoint = getExtensionPosition();

        //shuffleboard shit
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        ShuffleboardLayout pivotLayout = tab.getLayout("Pivot", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(0, 0);
        ShuffleboardLayout extensionLayout = tab.getLayout("Extension", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(2, 0);

        pivotLayout.addNumber("Pivot Setpoint (deg)", () -> pivotSetpoint);
        pivotLayout.addNumber("Pivot Angle (deg)", () -> getPivotAngle());

        extensionLayout.addNumber("Extension Setpoint (m)", () -> extensionSetpoint);
        extensionLayout.addNumber("Extension Position (m)", () -> getExtensionPosition());
    }

    public void pivot(boolean reversed) {
        pivotPID.setReference(3 * (reversed ? -1 : 1), ControlType.kVoltage);
        //pivotSetpoint += reversed ? -1 : 1;
    }

    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    public void stopPivot() {
        pivotPID.setReference(0, ControlType.kVelocity);
    }

    public void extend(boolean reversed) {
        extensionSetpoint += 1 / (2048.0 * 4) * (reversed ? -1 : 1);
        if(extensionSetpoint < 0) extensionSetpoint = 0;
    }

    /**
     * @return the position of the extension motor in meters
     */
    public double getExtensionPosition() {
        return extensionMotor.getSelectedSensorPosition() / (2048.0 * 4.0);
    }

    public void stopExtension() {
        extensionMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        //pivot
        //pivotPID.setReference(pivotSetpoint, ControlType.kPosition);

        //extension
        extensionMotor.set(ControlMode.Position, extensionSetpoint * 2048.0 * 4.0);
    }
}
