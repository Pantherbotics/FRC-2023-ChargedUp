package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Extend extends SubsystemBase {
    private final TalonFX motor;
    private double setpoint;

    private boolean isOpenLoop = false;
    
    public Extend() {
        // extension motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.peakOutputForward = 0.3;
        config.peakOutputReverse = -0.3;
        config.slot0.kP = ArmConstants.kPExtend;
        config.slot0.kI = ArmConstants.kIExtend;
        config.slot0.kD = ArmConstants.kDExtend;
        config.slot0.kF = ArmConstants.kFExtend;

        motor = new TalonFX(ArmConstants.kExtendMotorPort);
        motor.configAllSettings(config);
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        motor.setSelectedSensorPosition(0); // 0 position should be when the arm is fully down
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        
        setpoint = 0;  
    }
    /**
     * @param speed The percent output of the falcon
     */
    public void runOpenLoop(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * @param speed
     */
    public void runClosedLoop(double speed) {
        setPosition(setpoint + speed);
    }

    /**
     * @param position
     */
    public void setPosition(double position) {
        if(!withinBounds(position))
            return;
        setpoint = position;
    }

    /**
     * @param position The position to test
     * @return Whether the position is within the bounds of the extend (assuming the extend started in 0 position) 
     */
    private boolean withinBounds(double position) {
        return position >= ArmConstants.kExtendLowerBound && 
               position <= ArmConstants.kExtendUpperBound;
    }
    
    /**
     * Stops the extend motor
     */
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * @return the position of the extend motor
     */
    public double getPosition() {
        return motor.getSelectedSensorPosition();
    }

    /**
     * @return Whether the extend is in open loop control or not
     */
    public boolean getIsOpenLoop() {
        return isOpenLoop;
    }

    /**
     * @param openLoop True if is open loop, false if closed loop
     */
    public void setIsOpenLoop(boolean openLoop) {
        isOpenLoop = openLoop;
    }

    /**
     * @return The extend setpoint
     */
    public double getSetpoint() {
        return setpoint;
    }
    
    @Override
    public void periodic() {
        if(!isOpenLoop)
            motor.set(ControlMode.Position, setpoint);
    }
}
