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
        public static final double kDriveMotorGearRatio = 2/15D; // 12:30 then 15:45
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;

        //These were calculated for our swerve modules, but position PID based on the angle means they aren't needed
        //public static final double kTurningMotorGearRatio = 0.036; // 12:100 then 18:60
        //public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        //public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        public static final double kPDrive = 0.0001;
        public static final double kIDrive = 0.0;
        public static final double kDDrive = 0.0001;
        public static final double kIZoneDrive = 0.0;
        public static final double kFFDrive = 0.000175;

        //These values will need to be calibrated when swapping between CANCoders and Potentiometers
        public static final double kPTurning = 1.0;
        public static final double kITurning = 0.0005;
        public static final double kDTurning = 0.0;
        public static final double kFTurning = 0.0;
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

        //These are all id numbers, change if necessary (ie: you changed the motor ids for some reason)
        public static final int kFrontLeftModuleID = 1;
        public static final int kFrontRightModuleID = 2;
        public static final int kBackRightModuleID = 3;
        public static final int kBackLeftModuleID = 4;

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 4;

        public static final int kFrontLeftTurningEncoderPort = 5;
        public static final int kFrontRightTurningEncoderPort = 6;
        public static final int kBackRightTurningEncoderPort = 7;
        public static final int kBackLeftTurningEncoderPort = 8;

        //Encoders are obviously going to be at an angle when they are installed, figure them out through trial and error
        //Positive is counterclockwise, Negative is clockwise
        public static final int kFrontLeftTurningEncoderOffsetDeg = 165; //165  <-- I have no fucking clue what these commented numbers mean
        public static final int kFrontRightTurningEncoderOffsetDeg = 225; //225
        public static final int kBackRightTurningEncoderOffsetDeg = 350; //90
        public static final int kBackLeftTurningEncoderOffsetDeg = 163; //-20 

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
        public static final int kPrimaryJoystickLeftXID = 0;
        public static final int kPrimaryJoystickLeftYID = 1;
        public static final int kPrimaryJoystickRightXID = 4;
        public static final int kPrimaryJoystickRightYID = 5;

        public static final int kSecondaryJoystickID = 1;

        public static final double driverEXP = 7.0 / 3; //Exponentiate the joystick values to have finer control at low values

        public static final double kDeadband = 0.04; //Higher than average on the controller I'm using
    }
}