package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Pivot extends SubsystemBase {
    private final CANSparkMax master, slave;
    private final CANCoder cancoder;
    private final PIDController pid;

    public boolean openLoop = false;

    public Pivot() {
        master = new CANSparkMax(ArmConstants.kPivotLeaderMotorPort, MotorType.kBrushless);
        master.restoreFactoryDefaults();
        master.setIdleMode(IdleMode.kBrake);

        slave = new CANSparkMax(ArmConstants.kPivotFollowerMotorPort, MotorType.kBrushless);
        slave.restoreFactoryDefaults();
        slave.setIdleMode(IdleMode.kBrake);

        slave.follow(master, true);

        // cancoder
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = -97.703; 

        cancoder = new CANCoder(ArmConstants.kPivotCANCoderPort);
        cancoder.configAllSettings(config);

        pid = new PIDController(ArmConstants.kPPivot, ArmConstants.kIPivot, ArmConstants.kDPivot);
        pid.setSetpoint(getAngle());
        
        setAngle(ArmConstants.kPivotZeroAngle);
    }

    /**
     * Pivots the arm open loop
     * @param speed The desired speed to set the motor to, [-1, 1]
     */
    public void pivotOpenLoop(double speed) {
        master.set(speed);
    }

    /**
     * Pivots the arm closed loop; adds the speed to the setpoint
     * @param speed The speed in deg/s
     */
    public void pivotClosedLoop(double speed) {
        setAngle(pid.getSetpoint() + speed);
    }

    /**
     * @param angle The desired position to set the pivot to
     */
    public void setAngle(double angle) {
        if(!withinBounds(angle))
            return;
        pid.setSetpoint(angle);
    }

    /**
     * @param position The position to test
     * @return Whether the position is within the bounds of the pivot
     */
    private boolean withinBounds(double position) {
        return position >= ArmConstants.kPivotLowerBound && 
               position <= ArmConstants.kPivotUpperBound;
    }

    /**
     * Stops the pivot motor
     */
    public void stop() {
        master.stopMotor();
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
        return pid.getSetpoint();
    }

    @Override
    public void periodic()
    {
        if(!openLoop)
            master.set(MathUtil.clamp(pid.calculate(getAngle()), -0.3, 0.3));
    }
}
