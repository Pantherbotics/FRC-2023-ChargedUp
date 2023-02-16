package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class SwerveModule {
    private final SwerveSpeedController driveController;
    private final SwerveSteerController turnController;

    public SwerveModule(int driveMotorPort, int turnMotorPort, int cancoderPort, double cancoderOffset, ShuffleboardContainer container) {
        driveController = new SwerveSpeedController(driveMotorPort, container);
        turnController = new SwerveSteerController(turnMotorPort, cancoderPort, cancoderOffset, container);
    }

    public double getDriveVelocity() {
        return driveController.getVelocity();
    }

    public double getDrivePosition() {
        return driveController.getPosition();
    }

    public Rotation2d getTurnAngle() {
        return Rotation2d.fromDegrees(turnController.getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurnAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurnAngle());
    }

    public void setDesiredState(SwerveModuleState moduleState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(moduleState, getTurnAngle());
        driveController.setVelocity(optimizedState.speedMetersPerSecond);
        turnController.setAngle(optimizedState.angle.getDegrees());
    }

    public void setIdleMode(IdleMode mode) {
        driveController.setIdleMode(mode);
        turnController.setNeutralMode((mode.equals(IdleMode.kBrake)) ? NeutralMode.Brake : NeutralMode.Coast);
    }
}
