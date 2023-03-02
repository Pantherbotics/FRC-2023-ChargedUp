package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax pivotLeader, pivotFollower;
    private final CANCoder pivotCancoder;
    private final PIDController pivotPID;

    public boolean pivotOpenLoop = false;

    // extension
    private final TalonFX extensionMotor;
    private double extensionSetpoint;

    public boolean extendOpenLoop = false;

    public Arm() {
        pivotLeader = new CANSparkMax(ArmConstants.kPivotLeaderMotorPort, MotorType.kBrushless);
        pivotLeader.restoreFactoryDefaults();
        pivotLeader.setIdleMode(IdleMode.kBrake);

        pivotFollower = new CANSparkMax(ArmConstants.kPivotFollowerMotorPort, MotorType.kBrushless);
        pivotFollower.restoreFactoryDefaults();
        pivotFollower.setIdleMode(IdleMode.kBrake);

        pivotFollower.follow(pivotLeader, true);

        // cancoder
        CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        cancoderConfig.magnetOffsetDegrees = 0; // change

        pivotCancoder = new CANCoder(ArmConstants.kPivotCANCoderPort);
        pivotCancoder.configAllSettings(cancoderConfig);
        pivotCancoder.setPositionToAbsolute();
        pivotCancoder.setPosition(0);

        pivotPID = new PIDController(ArmConstants.kPPivot, ArmConstants.kIPivot, ArmConstants.kDPivot);

        // extension motor
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.slot0.kP = ArmConstants.kPExtend;
        motorConfig.slot0.kI = ArmConstants.kIExtend;
        motorConfig.slot0.kD = ArmConstants.kDExtend;
        motorConfig.slot0.kF = ArmConstants.kFExtend;

        extensionMotor = new TalonFX(ArmConstants.kExtensionMotorPort);
        extensionMotor.configAllSettings(motorConfig);
        extensionMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        extensionMotor.setSelectedSensorPosition(0); // 0 position should be when the arm is fully down
        extensionMotor.setNeutralMode(NeutralMode.Coast);
        extensionMotor.setInverted(TalonFXInvertType.CounterClockwise);
        
        extensionSetpoint = 0;  
    }

    public void pivotOpenLoop(double speed) {
        pivotLeader.set(speed);
    }

    public void pivotClosedLoop(double speed) {
        setPivotPosition(pivotPID.getSetpoint() + speed);
    }

    public void setPivotPosition(double position) {
        if(withinPivotBounds(position))
            pivotPID.setSetpoint(position);
    }

    public boolean withinPivotBounds(double position) {
        return true;
    }

    public double getPivotAngle() {
        return pivotCancoder.getPosition();
    }

    public void stopPivot() {
        pivotLeader.stopMotor();
        //pivotPID.setReference(0, ControlType.kVelocity);
    }

    public void extendClosedLoop(double speed) {
        setExtendPosition(extensionSetpoint + speed);
    }

    public void setExtendPosition(double position) {
        if(withinExtendBounds(position))
            extensionSetpoint = position;
    }

    public boolean withinExtendBounds(double position) {
        return position >= ArmConstants.kExtendLowerBound && 
               position <= ArmConstants.kExtendUpperBound;
    }

    /**
     * @return the position of the extension motor in meters
     */
    public double getExtensionPosition() {
        return extensionMotor.getSelectedSensorPosition();
    }

    public void stopExtension() {
        extensionMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        if(!pivotOpenLoop)
            pivotLeader.set(pivotPID.calculate(getPivotAngle()));
        
        if(!extendOpenLoop)
            extensionMotor.set(ControlMode.Position, extensionSetpoint);

        SmartDashboard.putNumber("Pivot Setpoint", pivotPID.getSetpoint());
        SmartDashboard.putNumber("Pivot Position", getPivotAngle());

        SmartDashboard.putNumber("Extension Setpoint", extensionSetpoint);
        SmartDashboard.putNumber("Extension Position", getExtensionPosition());
    }
}
