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
    private final TalonFX falcon;
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

        falcon = new TalonFX(ArmConstants.kExtendMotorPort);
        falcon.configAllSettings(config);
        falcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        falcon.setSelectedSensorPosition(0); // 0 position should be when the arm is fully down
        falcon.setNeutralMode(NeutralMode.Coast);
        falcon.setInverted(TalonFXInvertType.CounterClockwise);
        
        setpoint = 0;  
    }

    /**
     * @param speed The percent output of the falcon
     */
    public void runOpenLoop(double speed) {
        falcon.set(ControlMode.PercentOutput, speed);
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
     * @return the position of the extend motor
     */
    public double getPosition() {
        return falcon.getSelectedSensorPosition();
    }
    
    /**
     * @return The extend setpoint
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * @return Whether the extend is at (close to) its setpoint
     */
    public boolean atSetpoint() {
        return Math.abs(setpoint - getPosition()) < 7000;
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
        falcon.set(ControlMode.PercentOutput, 0);
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
    
    @Override
    public void periodic() {
        if(!isOpenLoop)
            falcon.set(ControlMode.Position, setpoint);
    }
}
