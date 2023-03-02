package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax flexMotor, rotateMotor;
    private final RelativeEncoder flexEncoder, rotateEncoder;
    private final SparkMaxPIDController flexPID, rotatePID;
    private double flexSetpoint, rotateSetpoint; //these should be zero at start

    public boolean flexOpenLoop = false, rotateOpenLoop = false;

    public Wrist() {
        // flex motor
        flexMotor = new CANSparkMax(ArmConstants.kFlexMotorPort, MotorType.kBrushless);
        flexMotor.restoreFactoryDefaults();
        flexMotor.setIdleMode(IdleMode.kCoast);

        // flex encoder
        flexEncoder = flexMotor.getEncoder();
        flexEncoder.setPositionConversionFactor(360); // 360 degrees per rotation
        flexEncoder.setVelocityConversionFactor(6); // 6 degrees per second
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
        rotateEncoder.setPositionConversionFactor(360);
        rotateEncoder.setVelocityConversionFactor(6);
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
        setFlexPosition(flexSetpoint + speed);
    }

    public void setFlexPosition(double position) {
        if(withinFlexBounds(position))
            flexSetpoint = position;
    }

    public double getFlexAngle() {
        return flexEncoder.getPosition();
    }

    public double getFlexSetpoint() {
        return flexSetpoint;
    }

    private boolean withinFlexBounds(double position) {
        return position >= ArmConstants.kFlexLowerBound && 
               position <= ArmConstants.kFlexUpperBound;
    }

    public void stopFlex() {
        flexMotor.stopMotor();
    }

    public void rotateOpenLoop(double speed) {
        rotateMotor.set(speed);
    }

    public void rotateClosedLoop(double speed) {
        setRotatePosition(rotateSetpoint + speed);
    }

    public void setRotatePosition(double position) {
        if(withinRotateBounds(position))
            rotateSetpoint = position;
    }

    private boolean withinRotateBounds(double position) {
        return position >= ArmConstants.kRotateLowerBound && 
               position <= ArmConstants.kRotateUpperBound;
    }

    public void stopRotate() {
        rotateMotor.stopMotor();
    }

    public double getRotateAngle() {
        return rotateEncoder.getPosition();
    }

    public double getRotateSetpoint() {
        return rotateSetpoint;
    }

    @Override
    public void periodic() {
        if(!flexOpenLoop)
            flexPID.setReference(flexSetpoint, ControlType.kPosition);
            
        if(!rotateOpenLoop)
            rotatePID.setReference(rotateSetpoint, ControlType.kPosition);
    }
}