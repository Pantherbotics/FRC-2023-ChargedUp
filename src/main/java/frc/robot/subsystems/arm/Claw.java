package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final CANSparkMax flexMotor, rotateMotor; //switch to not vex stuff

    private final CANCoder flexEncoder;
    private final PIDController flexPID;

    private final DoubleSolenoid clawSolenoid;

    private boolean doPID = false;

    public Claw() {
        //motors
        flexMotor = new CANSparkMax(11, MotorType.kBrushless);
        rotateMotor = new CANSparkMax(12, MotorType.kBrushless);

        //flex encoder
        flexEncoder = new CANCoder(1);
        flexEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        //flex pid
        flexPID = new PIDController(0.05, 0.1, 0);

        //solenoid, open/closes the claw
        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void flexPID(double speed) {
        boolean withinBounds = getFlexAbsolutePosition() < 485 && getFlexAbsolutePosition() > 253;
        double newSetpoint = flexPID.getSetpoint() + (withinBounds ? normalizeSpeed(speed) : 0);
        flexPID.setSetpoint(newSetpoint);
    }

    public double getFlexAbsolutePosition() {
        return flexEncoder.getAbsolutePosition() + (flexEncoder.getAbsolutePosition() < 250 ? 360 : 0);
    }
    

    private double normalizeSpeed(double speed) {
        double magnitude = Math.abs(speed);
        if(magnitude <= 0.3) return 0;
        if(magnitude > 1) return speed / magnitude;
        return speed;
    }

    public boolean getDoPID() {
        return doPID;
    }

    public void setDoPID(boolean doPID) {
        this.doPID = doPID;
    }

    public void stop() {
        flexMotor.set(0);
        rotateMotor.set(0);
    }

    @Override
    public void periodic() {
        if(doPID) 
        {
            
        }

        SmartDashboard.putNumber("Flex point", flexPID.getSetpoint());

        SmartDashboard.putNumber("Flex Encoder", getFlexAbsolutePosition());
        SmartDashboard.putNumber("Raw Flex Encoder", flexEncoder.getAbsolutePosition());

        SmartDashboard.putNumber("Flex kP", 0);
        SmartDashboard.getNumber("Flex kP", 0);

        SmartDashboard.putNumber("Flex kI", 0);
        SmartDashboard.getNumber("Flex kI", 0);

        SmartDashboard.putNumber("Flex kD", 0);
        SmartDashboard.getNumber("Flex kD", 0);
    }
}
