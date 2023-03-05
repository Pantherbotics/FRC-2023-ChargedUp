package frc.robot.subsystems.arm;

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
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class Pivot extends PIDSubsystem {
    private final CANSparkMax masterMotor, slaveMotor;
    private final CANCoder cancoder;
    private final PIDController pivotPID;

    public boolean openLoop = false;

    public Pivot() {
        super(new PIDController(ArmConstants.kPPivot, ArmConstants.kIPivot, ArmConstants.kDPivot));

        masterMotor = new CANSparkMax(ArmConstants.kPivotLeaderMotorPort, MotorType.kBrushless);
        masterMotor.restoreFactoryDefaults();
        masterMotor.setIdleMode(IdleMode.kBrake);

        slaveMotor = new CANSparkMax(ArmConstants.kPivotFollowerMotorPort, MotorType.kBrushless);
        slaveMotor.restoreFactoryDefaults();
        slaveMotor.setIdleMode(IdleMode.kBrake);

        slaveMotor.follow(masterMotor, true);

        // cancoder
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = -97.703; 

        cancoder = new CANCoder(ArmConstants.kPivotCANCoderPort);
        cancoder.configAllSettings(config);

        pivotPID = new PIDController(ArmConstants.kPPivot, ArmConstants.kIPivot, ArmConstants.kDPivot);
        pivotPID.setSetpoint(getAngle());
        
        setAngle(ArmConstants.kPivotZeroAngle);
    }

    /**
     * Pivots the arm open loop
     * @param speed The desired speed to set the motor to, [-1, 1]
     */
    public void pivotOpenLoop(double speed) {
        masterMotor.set(speed);
    }

    /**
     * Pivots the arm closed loop; adds the speed to the setpoint
     * @param speed The speed in deg/s
     */
    public void pivotClosedLoop(double speed) {
        setAngle(pivotPID.getSetpoint() + speed);
    }

    /**
     * @param angle The desired position to set the pivot to
     */
    public void setAngle(double angle) {
        if(!withinBounds(angle))
            return;
        pivotPID.setSetpoint(angle);
    }

    /**
     * @param position The position to test
     * @return Whether the position is within the bounds of the pivot
     */
    private boolean withinBounds(double position) {
        return true;//position >= ArmConstants.kPivotLowerBound && 
               //position <= ArmConstants.kPivotUpperBound;
    }

    /**
     * Stops the pivot motor
     */
    public void stop() {
        masterMotor.stopMotor();
    }

    /**
     * @return The pivot angle in deg
     */
    public double getAngle() {
        return cancoder.getPosition();
    }

    /**
     * @return The pivot setpoint in deg
     */
    public double getSetpoint() {
        return pivotPID.getSetpoint();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }
    
}
