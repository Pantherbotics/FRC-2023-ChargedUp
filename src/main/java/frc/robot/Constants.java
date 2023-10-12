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

        //Positive is counterclockwise, Negative is clockwise
        public static final double kFrontLeftCANCoderOffsetDeg = 0.5; 
        public static final double kFrontRightCANCoderOffsetDeg = -17.00; 
        public static final double kBackRightCANCoderOffsetDeg = 80.56; 
        public static final double kBackLeftCANCoderOffsetDeg = -201; 

        //Whether to invert the drive motor 
        public static final boolean kFrontLeftDriveMotorInverted = true; 
        public static final boolean kFrontRightDriveMotorInverted = false; 
        public static final boolean kBackRightDriveMotorInverted = false; 
        public static final boolean kBackLeftDriveMotorInverted = true; 
    }
    
    public static final class ArmConstants {
        //pivot
        public static final double kPivotCANCoderOffsetDeg = -262.936;
        
        public static final double kPivotZeroAngle = 83.496;

        public static final double kPivotUpperBound = 360;
        public static final double kPivotLowerBound = 0;

        public static final double kPPivot = 0.02;
        public static final double kIPivot = 0.0;
        public static final double kDPivot = 0.0;

        public static final int kPivotLeaderMotorPort = 5;
        public static final int kPivotFollowerMotorPort = 6;

        public static final int kPivotCANCoderPort = 9;

        //extend
        public static final double kExtendUpperBound = 52000;
        public static final double kExtendLowerBound = 0;

        public static final double kPExtend = 0.05;
        public static final double kIExtend = 0.0;
        public static final double kDExtend = 0.0;
        public static final double kFExtend = 0.0;
        
        
        public static final int kExtendMotorPort = 5;

        //flex
        public static final double kFlexLowerBound = -300;
        public static final double kFlexUpperBound = 300;

        public static final double kPFlex = 0.05;
        public static final double kIFlex = 0.0;
        public static final double kDFlex = 0.0;
        public static final double kIZoneFlex = 0.0;
        public static final double kFFFlex = 0.0;

        public static final int kFlexMotorPort = 8;

        //rotate
        public static final double kRotateLowerBound = -28.5;
        public static final double kRotateUpperBound = 0;

        public static final double kPRotate = 0.05;
        public static final double kIRotate = 0.0;
        public static final double kDRotate = 0.0;
        public static final double kIZoneRotate = 0.0;
        public static final double kFFRotate = 0.0;

        public static final int kRotateMotorPort = 7;

        //claw 
        public static final int kClawSolenoidPort = 0;
    }

    public static final class VisionConstants {
        public static final double kLimelightV1FOVAngle = 27.0;
        public static final double kLimeLightV2FOVAngle = 29.8;
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

        public static final double kPXYController = 1.5;
        public static final double kIXYController = 0;
        public static final double kDXYController = 0;
        
        public static final double kPThetaController = 3.0;
        public static final double kIThetaController = 0;
        public static final double kDThetaController = 0;

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
        public static final int kPrimaryJoystickLeftTriggerID = 2;
        public static final int kPrimaryJoystickRightTriggerID = 3;
        public static final int kPrimaryJoystickRightXAxisID = 4;
        public static final int kPrimaryJoystickRightYAxisID = 5;

        public static final int kSecondaryJoystickID = 1;
        public static final int kSecondaryJoystickLeftXAxisID = 0;
        public static final int kSecondaryJoystickLeftYAxisID = 1;
        public static final int kSecondaryJoystickLeftTriggerID = 2;
        public static final int kSecondaryJoystickRightTriggerID = 3;
        public static final int kSecondaryJoystickRightXAxisID = 4;
        public static final int kSecondaryJoystickRightYAxisID = 5;

        public static final double kDriverExp = 7.0 / 3; //Exponentiate the joystick values to have finer control at low values

        public static final double kDeadband = 0.01; 
    }
}