package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax flexMotor;
    private final RelativeEncoder flexEncoder;
    private final SparkMaxPIDController flexPID;
    private double flexSetpoint; //these should be zero at start

    private boolean isFlexOpenLoop = false, isRotateOpenLoop = false;

    public Wrist() {
        // flex motor
        flexMotor = new CANSparkMax(ArmConstants.kFlexMotorPort, MotorType.kBrushless);
        flexMotor.restoreFactoryDefaults();
        flexMotor.setIdleMode(IdleMode.kCoast);

        // flex encoder
        flexEncoder = flexMotor.getEncoder();
        flexEncoder.setPosition(0);

        // flex pid
        flexPID = flexMotor.getPIDController();
        flexPID.setP(ArmConstants.kPFlex);
        flexPID.setI(ArmConstants.kIFlex);
        flexPID.setD(ArmConstants.kDFlex);
        flexPID.setIZone(ArmConstants.kIZoneFlex);
        flexPID.setFF(ArmConstants.kFFFlex);
        flexPID.setOutputRange(-1, 1);
    }

    /**
     * Flex control in open loop
     * @param speed the speed to set it to [-1, 1]
     */
    public void flexOpenLoop(double speed) {
        flexMotor.set(speed);
    }

    /**
     * Flex control in closed loop
     * @param speed The speed in rpm
     */
    public void flexClosedLoop(double speed) {
        double newPos = flexSetpoint + speed;
        if(newPos < Constants.ArmConstants.kFlexReverseLimit){
            newPos = Constants.ArmConstants.kFlexReverseLimit;
        }
        if(newPos > Constants.ArmConstants.kFlexForwardLimit){
            newPos = Constants.ArmConstants.kFlexForwardLimit;
        }
        setFlexAngle(newPos);
    }

    /**
     * Sets the flex angle
     * @param angle The angle of the flex in rotations
     */
    public void setFlexAngle(double angle) {
        if(!withinFlexBounds(angle))
            return;
        flexSetpoint = angle;
    }

    /**
     * @return The angle of the flex in rotations
     */
    public double getFlexAngle() {
        return flexEncoder.getPosition();
    }

    /**
     * @return The setpoint of the flex in rotations
     */
    public double getFlexSetpoint() {
        return flexSetpoint;
    }

    public boolean atFlexSetpoint() {
        return Math.abs(flexSetpoint - getFlexAngle()) < 3;
    }

    /**
     * @param angle The angle to test
     * @return Whether the angle is within the flex bounds
     */
    private boolean withinFlexBounds(double angle) {
        return angle >= ArmConstants.kFlexLowerBound && 
               angle <= ArmConstants.kFlexUpperBound;
    }

    public void stopFlex() {
        flexMotor.stopMotor();
    }

    /**
     * @return Whether the flex is in open loop control or not
     */
    public boolean getIsFlexOpenLoop() {
        return isFlexOpenLoop;
    }

    /**
     * @param openLoop True if is open loop, false if closed loop
     */
    public void setIsFlexOpenLoop(boolean openLoop) {
        isFlexOpenLoop = openLoop;
    }

    private boolean withinRotateBounds(double angle) {
        return angle >= ArmConstants.kRotateLowerBound && 
               angle <= ArmConstants.kRotateUpperBound;
    }

    @Override
    public void periodic() {
        if(!isFlexOpenLoop)
            flexPID.setReference(flexSetpoint, ControlType.kPosition);
    }
}