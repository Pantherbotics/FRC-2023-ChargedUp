package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final PWMSparkMax[] flexMotors, rotateMotors; //switch to not vex stuff

    private final CANCoder flexEncoder, rotateEncoder;
    private final PIDController flexPID, rotatePID;

    private final DoubleSolenoid clawSolenoid;

    private boolean doPID = false;

    public Claw() {
        //flex motors (4)
        flexMotors = new PWMSparkMax[] {
            new PWMSparkMax(0),
            new PWMSparkMax(1),
            new PWMSparkMax(2),
            new PWMSparkMax(3),
        };

        //flex encoder
        flexEncoder = new CANCoder(1);
        flexEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        //flex pid
        flexPID = new PIDController(0.05, 0.1, 0);
        
        //rotation motors (2)
        rotateMotors = new PWMSparkMax[] {
            new PWMSparkMax(4),
            new PWMSparkMax(5)
        };

        //rotation encoder
        rotateEncoder = new CANCoder(0);

        //rotation pid
        rotatePID = new PIDController(0.8, 0.2, 0);

        //solenoid, open/closes the claw
        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void flexPID(double speed) {
        boolean withinBounds = getFlexAbsolutePosition() < 485 && getFlexAbsolutePosition() > 253;
        double newSetpoint = flexPID.getSetpoint() + (withinBounds ? normalizeSpeed(speed) : 0);
        flexPID.setSetpoint(newSetpoint);
    }

    public void flexOpenLoop(double speed) {
        setFlexMotors(normalizeSpeed(speed));
    }

    public double getFlexAbsolutePosition() {
        return flexEncoder.getAbsolutePosition() + (flexEncoder.getAbsolutePosition() < 250 ? 360 : 0);
    }

    private void setFlexMotors(double speed) {
        for(PWMSparkMax motor : flexMotors) {
            motor.set(speed);
        }
    }
    
    public void rotatePID(double speed) {
        double newSetpoint = flexPID.getSetpoint() + normalizeSpeed(speed);
        rotatePID.setSetpoint(newSetpoint);
    }
    
    public void rotateOpenLoop(double speed) {
        setRotateMotors(normalizeSpeed(speed));
    }

    public double getRotateAbsolutePosition() {
        return rotateEncoder.getAbsolutePosition();
    }

    private void setRotateMotors(double speed) {
        for(PWMSparkMax motor : rotateMotors) {
            //motor.set(speed);
        }
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
        setFlexMotors(0);
        setRotateMotors(0);
    }

    @Override
    public void periodic() {
        if(doPID) 
        {
            setFlexMotors(flexPID.calculate(getFlexAbsolutePosition()));
            setRotateMotors(rotatePID.calculate(getRotateAbsolutePosition()));
        }

        SmartDashboard.putNumber("Flex point", flexPID.getSetpoint());
        SmartDashboard.putNumber("Rotate point", rotatePID.getSetpoint());

        SmartDashboard.putNumber("Flex Encoder", getFlexAbsolutePosition());
        SmartDashboard.putNumber("Raw Flex Encoder", flexEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Rotate Encoder", rotateEncoder.getAbsolutePosition());

        SmartDashboard.putNumber("Flex kP", 0);
        SmartDashboard.getNumber("Flex kP", 0);

        SmartDashboard.putNumber("Flex kI", 0);
        SmartDashboard.getNumber("Flex kI", 0);

        SmartDashboard.putNumber("Flex kD", 0);
        SmartDashboard.getNumber("Flex kD", 0);
    }
}
