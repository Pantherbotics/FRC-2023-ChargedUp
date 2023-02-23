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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax pivotLeaderTest, pivotFollowerTest;//pivotLeader, pivotFollower;
    private final CANCoder pivotCancoder;
    private final PIDController pivotPID;

    public boolean pivotOpenLoop = true;

    // extension
    private final TalonFX extensionMotor;
    private double extensionSetpoint;

    public Arm() {
        pivotLeaderTest = new CANSparkMax(ArmConstants.kPivotLeaderMotorPort, MotorType.kBrushless);
        pivotLeaderTest.restoreFactoryDefaults();

        pivotFollowerTest = new CANSparkMax(ArmConstants.kPivotFollowerMotorPort, MotorType.kBrushless);
        pivotFollowerTest.restoreFactoryDefaults();

        pivotFollowerTest.follow(pivotLeaderTest, true);

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
        motorConfig.slot0.kP = ArmConstants.kPExtension;
        motorConfig.slot0.kI = ArmConstants.kIExtension;
        motorConfig.slot0.kD = ArmConstants.kDExtension;
        motorConfig.slot0.kF = ArmConstants.kFExtension;
        // motorConfig.forwardSoftLimitThreshold = 1000000;
        // motorConfig.reverseSoftLimitThreshold = 0;
        // motorConfig.forwardSoftLimitEnable = false;
        // motorConfig.reverseSoftLimitEnable = false;

        extensionMotor = new TalonFX(ArmConstants.kExtensionMotorPort);
        extensionMotor.configAllSettings(motorConfig);
        extensionMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        extensionMotor.setSelectedSensorPosition(0); // 0 position should be when the arm is fully down
        extensionMotor.setNeutralMode(NeutralMode.Coast);
        extensionMotor.setInverted(TalonFXInvertType.CounterClockwise);
        
        extensionSetpoint = 0;  

        //shuffleboard shit
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        ShuffleboardLayout pivotLayout = tab.getLayout("Pivot", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(0, 0);
        ShuffleboardLayout extensionLayout = tab.getLayout("Extension", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(2, 0);

        pivotLayout.addNumber("Setpoint (deg)", () -> pivotPID.getSetpoint());
        pivotLayout.addNumber("Angle (deg)", () -> getPivotAngle());
        pivotLayout.addBoolean("Open Loop?", () -> pivotOpenLoop);

        extensionLayout.addNumber("Setpoint (m)", () -> extensionSetpoint);
        extensionLayout.addNumber("Position (m)", () -> getExtensionPosition());
    }

    public void pivotClosedLoop(double speed) {
        double newSetpoint = pivotPID.getSetpoint() + speed;
        pivotPID.setSetpoint(newSetpoint);
    }

    public void pivotOpenLoop(double speed) {
        pivotLeaderTest.set(speed);
    }

    public double getPivotAngle() {
        return pivotCancoder.getPosition();
    }

    public void stopPivot() {
        pivotLeaderTest.stopMotor();
        //pivotPID.setReference(0, ControlType.kVelocity);
    }

    public void extend(double speed) {
        extensionSetpoint += speed;
        if(extensionSetpoint < 0) extensionSetpoint = 0;
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
            pivotLeaderTest.set(pivotPID.calculate(getPivotAngle()));
        //extensionMotor.set(ControlMode.PercentOutput, .3);
        extensionMotor.set(ControlMode.Position, extensionSetpoint);
    }
}
