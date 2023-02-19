package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    //--------------------------------------------------------------------------------------------
    //      Notes:
    //-If we get new wheels, edit kWheelDiameterMeters
    //--------------------------------------------------------------------------------------------

    //If using the Potentiometer, this specifies its max value so we can get the angle
    public static final double potMax = 3798;

    public static final double neoMaxRPM = 5000; //5000 was experimentally determined from our swerve chassis


    //Checked and verified as of May 1st, 2022
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //4 when new
        public static final double kDriveMotorGearRatio = 2.0 / 15; // 12:30 then 15:45
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI;
        public static final double kDriveEncoderRPM2MetersPerSec = kDriveEncoderRot2Meter / 60;

        public static final double kDriveVelocityCoefficient = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / neoMaxRPM;
        public static final double kTurnPositionCoefficient = 360.0 / 4069.0;
        
        //These were calculated for our swerve modules, but position PID based on the angle means they aren't needed
        //public static final double kTurningMotorGearRatio = 0.036; // 12:100 then 18:60
        //public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        //public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        //drive pid values
        public static final double kPDrive = 0.0001;
        public static final double kIDrive = 0.0;
        public static final double kDDrive = 0.0001;
        public static final double kIZoneDrive = 0.0;
        public static final double kFFDrive = 0.000175;

        //turn pid values
        public static final double kPTurn = 1.0;
        public static final double kITurn = 0.0005;
        public static final double kDTurn = 0.0;
        public static final double kFTurn = 0.0;

        //These are all id numbers, change if necessary (ie: you reconfigured the motor ids for some reason)
        public static final int kFrontLeftModuleID = 1;
        public static final int kFrontRightModuleID = 2;
        public static final int kBackRightModuleID = 3;
        public static final int kBackLeftModuleID = 4;

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 4;

        public static final int kFrontLeftTurnMotorPort = 1;
        public static final int kFrontRightTurnMotorPort = 2;
        public static final int kBackRightTurnMotorPort = 3;
        public static final int kBackLeftTurnMotorPort = 4;

        public static final int kFrontLeftCANCoderPort = 5;
        public static final int kFrontRightCANCoderPort = 6;
        public static final int kBackRightCANCoderPort = 7;
        public static final int kBackLeftCANCoderPort = 8;

        //Positive is counterclockwise, Negative is clockwise
        public static final int kFrontLeftCANCoderOffsetDeg = 0; 
        public static final int kFrontRightCANCoderOffsetDeg = 0; 
        public static final int kBackRightCANCoderOffsetDeg = 0; 
        public static final int kBackLeftCANCoderOffsetDeg = 0; 
    }
    
    public static final class ArmConstants {
        public static final double kPivotMotorGearRatio = 5.0 * 4 * 10;
        public static final double kPivotEncoderRot2Degrees = 360;
        public static final double kPivotEncoderRPM2DegreesPerSec = 6;

        public static final double kPFlex = 0.01;
        public static final double kIFlex = 0.0;
        public static final double kDFlex = 0.00;
        public static final double kIZoneFlex = 0;
        public static final double kFFFlex = 0;

        public static final double kPRotate = 0.7;
        public static final double kIRotate = 0.0;
        public static final double kDRotate = 0.00;
        public static final double kIZoneRotate = 0;
        public static final double kFFRotate = 0;

        public static final int kPivotLeaderMotorPort = 5;
        public static final int kPivotFollowerMotorPort = 6;

        public static final int kPivotCANCoderPort = 0;
        
        public static final int kExtensionMotorPort = 5;

        public static final int kCTREPCMid = 9;
    }

    //Checked and verified as of May 1st, 2022
    public static final class DriveConstants {
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(19.5);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(19.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d( kWheelBase / 2,  kTrackWidth / 2),  //Front Left
                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),  //Front Right
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),  //Back Right
                new Translation2d(-kWheelBase / 2,  kTrackWidth / 2)   //Back Left
        );

        public static final double kPhysicalMaxSpeedMetersPerSecond = (neoMaxRPM / 60.0) * ModuleConstants.kDriveEncoderRot2Meter; //~3.56 m/s
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI; //About 2pi given wheelbase and drive speed

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond; //we have about 11.68 ft/s, we don't need to reduce it
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond; //360 degrees per second doesn't need to be reduced

        //These values could be tuned:
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.5;
    }

    //Checked and verified as of May 1st, 2022
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond; //2pi
        //All the following Constants can be tuned:
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3.0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared
                    );
    }

    //Checked and verified as of May 1st, 2022
    public static final class OIConstants {
        public static final int kPrimaryJoystickID = 0;
        public static final int kPrimaryJoystickLeftXAxisID = 0;
        public static final int kPrimaryJoystickLeftYAxisID = 1;
        public static final int kPrimaryJoystickRightXAxisID = 4;
        public static final int kPrimaryJoystickRightYAxisID = 5;

        public static final int kSecondaryJoystickID = 1;
        public static final int kSecondaryJoystickLeftXAxisID = 0;
        public static final int kSecondaryJoystickLeftYAxisID = 1;
        public static final int kSecondaryJoystickRightXAxisID = 4;
        public static final int kSecondaryJoystickRightYAxisID = 5;

        public static final double kDriverExp = 7.0 / 3; //Exponentiate the joystick values to have finer control at low values

        public static final double kDeadband = 0.04; //Higher than average on the controller I'm using
    }
}