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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax pivotLeader, pivotFollower;
    private final CANCoder pivotCancoder;
    private final PIDController pivotPID;

    public boolean pivotOpenLoop = false;

    // extension
    private final TalonFX extendMotor;
    private double extendSetpoint;

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
        motorConfig.slot0.kP = ArmConstants.kPEntend;
        motorConfig.slot0.kI = ArmConstants.kIExtend;
        motorConfig.slot0.kD = ArmConstants.kDExtend;
        motorConfig.slot0.kF = ArmConstants.kFExtend;

        extendMotor = new TalonFX(ArmConstants.kExtendMotorPort);
        extendMotor.configAllSettings(motorConfig);
        extendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        extendMotor.setSelectedSensorPosition(0); // 0 position should be when the arm is fully down
        extendMotor.setNeutralMode(NeutralMode.Coast);
        extendMotor.setInverted(TalonFXInvertType.CounterClockwise);
        
        extendSetpoint = 0;  
    }

    /**
     * Pivots the arm open loop
     * @param speed The desired speed to set the motor to, [-1, 1]
     */
    public void pivotOpenLoop(double speed) {
        pivotLeader.set(speed);
    }

    /**
     * Pivots the arm closed loop; adds the speed to the setpoint
     * @param speed The speed in deg/s
     */
    public void pivotClosedLoop(double speed) {
        setPivotPosition(pivotPID.getSetpoint() + speed);
    }

    /**
     * @param position The desired position to set the pivot to
     */
    public void setPivotPosition(double position) {
        if(withinPivotBounds(position))
            pivotPID.setSetpoint(position);
    }

    /**
     * @param position The position to test
     * @return Whether the position is within the bounds of the pivot
     */
    private boolean withinPivotBounds(double position) {
        return true;
        //TODO
    }

    /**
     * Stops the pivot motor
     */
    public void stopPivot() {
        pivotLeader.stopMotor();
    }

    /**
     * @return The pivot angle in deg
     */
    public double getPivotAngle() {
        return pivotCancoder.getPosition();
    }

    /**
     * @return The pivot setpoint in deg
     */
    public double getPivotSetpoint() {
        return pivotPID.getSetpoint();
    }

    /**
     * @param speed
     */
    public void extendClosedLoop(double speed) {
        setExtendPosition(extendSetpoint + speed);
    }

    /**
     * @param position
     */
    public void setExtendPosition(double position) {
        if(withinExtendBounds(position))
            extendSetpoint = position;
    }

    /**
     * @param position The position to test
     * @return Whether the position is within the bounds of the extend (assuming the extend started in 0 position) 
     */
    private boolean withinExtendBounds(double position) {
        return position >= ArmConstants.kExtendLowerBound && 
               position <= ArmConstants.kExtendUpperBound;
    }
    
    /**
     * Stops the extend motor
     */
    public void stopExtension() {
        extendMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * @return the position of the extend motor
     */
    public double getExtendPosition() {
        return extendMotor.getSelectedSensorPosition();
    }

    /**
     * @return The extend setpoint
     */
    public double getExtendSetpoint() {
        return extendSetpoint;
    }

    @Override
    public void periodic() {
        if(!pivotOpenLoop)
            pivotLeader.set(pivotPID.calculate(getPivotAngle()));
        
        if(!extendOpenLoop)
            extendMotor.set(ControlMode.Position, extendSetpoint);
    }
}
