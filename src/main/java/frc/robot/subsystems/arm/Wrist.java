package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax flexMotor, rotateMotor;
    private final RelativeEncoder flexEncoder, rotateEncoder;
    private final SparkMaxPIDController flexPID, rotatePID;
    private double flexSetpoint, rotateSetpoint; //these should be zero at start

    public Wrist() {
        // flex motor
        flexMotor = new CANSparkMax(ArmConstants.kFlexMotorPort, MotorType.kBrushless);
        flexMotor.restoreFactoryDefaults();
        flexMotor.setIdleMode(IdleMode.kCoast);
        flexMotor.setSoftLimit(SoftLimitDirection.kForward, -1);
        flexMotor.setSoftLimit(SoftLimitDirection.kReverse, 260);
        flexMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        flexMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

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

        flexMotor.burnFlash();

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

        rotateMotor.burnFlash();

        rotateSetpoint = 0;

        // shuffleboard stuff
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        ShuffleboardLayout flexLayout = tab.getLayout("Flex", BuiltInLayouts.kList)
            .withSize(1, 1)
            .withPosition(4, 0);
        ShuffleboardLayout rotateLayout = tab.getLayout("Rotate", BuiltInLayouts.kList)
            .withSize(1, 1)
            .withPosition(4, 1);
        
        flexLayout.addNumber("Angle (deg)", () -> getFlexAngle());
        rotateLayout.addNumber("Angle (deg)", () -> getRotateAngle());
    }

    public void flexOpenLoop(double speed) {
        flexMotor.set(speed);
    }

    public void setFlexAngle(double speed) {
        flexSetpoint += speed;
        //flexPID.setReference(output * 5, ControlType.kVoltage);
    }

    public double getFlexAngle() {
        return flexEncoder.getPosition();
    }

    public void rotateOpenLoop(double speed) {
        rotateMotor.set(speed);
    }

    public void setRotateAngle(double speed) {
        rotateSetpoint += speed;
        //rotatePID.setReference(output * 3, ControlType.kVoltage);
    }

    public double getRotateAngle() {
        return rotateEncoder.getPosition();
    }

    public void stop() {
        flexPID.setReference(0, ControlType.kVelocity);
        rotatePID.setReference(0, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        // the setpoints are in degrees
        flexPID.setReference(flexSetpoint, ControlType.kPosition);
        rotatePID.setReference(rotateSetpoint, ControlType.kPosition);
    }
}