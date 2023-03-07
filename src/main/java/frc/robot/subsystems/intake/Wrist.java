package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax flexMotor, rotateMotor;
    private final RelativeEncoder flexEncoder, rotateEncoder;
    private final SparkMaxPIDController flexPID, rotatePID;
    private double flexSetpoint, rotateSetpoint; //these should be zero at start

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

        flexSetpoint = 0;

        // rotate motor
        rotateMotor = new CANSparkMax(ArmConstants.kRotateMotorPort, MotorType.kBrushless);
        rotateMotor.restoreFactoryDefaults();
        rotateMotor.setIdleMode(IdleMode.kCoast);

        // rotate encoder
        rotateEncoder = rotateMotor.getEncoder();
        rotateEncoder.setPosition(0);

        // rotate pid
        rotatePID = rotateMotor.getPIDController();
        rotatePID.setP(ArmConstants.kPRotate);
        rotatePID.setI(ArmConstants.kIRotate);
        rotatePID.setD(ArmConstants.kDRotate);
        rotatePID.setIZone(ArmConstants.kIZoneRotate);
        rotatePID.setFF(ArmConstants.kFFRotate);
        rotatePID.setOutputRange(-1, 1);

        rotateSetpoint = 0;
    }

    public void flexOpenLoop(double speed) {
        flexMotor.set(speed);
    }

    public void flexClosedLoop(double speed) {
        setFlexAngle(flexSetpoint + speed);
    }

    public void setFlexAngle(double angle) {
        if(!withinFlexBounds(angle))
            return;
        flexSetpoint = angle;
    }

    public double getFlexAngle() {
        return flexEncoder.getPosition();
    }

    public double getFlexSetpoint() {
        return flexSetpoint;
    }

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

    public void rotateOpenLoop(double speed) {
        rotateMotor.set(speed);
    }

    public void rotateClosedLoop(double speed) {
        setRotateAngle(rotateSetpoint + speed);
    }

    public void setRotateAngle(double angle) {
        if(!withinRotateBounds(angle))
            return;
        rotateSetpoint = angle;
    }

    public double getRotateAngle() {
        return rotateEncoder.getPosition();
    }

    public double getRotateSetpoint() {
        return rotateSetpoint;
    }

    private boolean withinRotateBounds(double angle) {
        return angle >= ArmConstants.kRotateLowerBound && 
               angle <= ArmConstants.kRotateUpperBound;
    }

    public void stopRotate() {
        rotateMotor.stopMotor();
    }

    /**
     * @return Whether the rotate is in open loop control or not
     */
    public boolean getIsRotateOpenLoop() {
        return isRotateOpenLoop;
    }

    /**
     * @param openLoop True if is open loop, false if closed loop
     */
    public void setIsRotateOpenLoop(boolean openLoop) {
        isRotateOpenLoop = openLoop;
    }

    @Override
    public void periodic() {
        if(!isFlexOpenLoop)
            flexPID.setReference(flexSetpoint, ControlType.kPosition);
            
        if(!isRotateOpenLoop)
            rotatePID.setReference(rotateSetpoint, ControlType.kPosition);
    }
}