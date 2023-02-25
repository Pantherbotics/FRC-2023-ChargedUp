package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    @SuppressWarnings("unused")
    public enum EncoderType {
        Potentiometer, CanCoder
    }
    //--------------------------------------------------------------------------------------------
    //      Notes:
    //-If we get new wheels, edit kWheelDiameterMeters
    //--------------------------------------------------------------------------------------------

    /**
     * The encoder type we are using for the swerve
     * Options: CanCoder, Potentiometer
     */
    public static final EncoderType kEncoderType = EncoderType.CanCoder;
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
                new Translation2d( kWheelBase / 2,  kTrackWidth / 2),  //Left Front
                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),  //Right Front
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),  //Right Back
                new Translation2d(-kWheelBase / 2,  kTrackWidth / 2)   //Left Back
        );

        public static final double kPhysicalMaxSpeedMetersPerSecond = (neoMaxRPM/60D) * ModuleConstants.kDriveEncoderRot2Meter; //~3.56 m/s
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = Math.PI*2; //About 2PI given wheelbase and drive speed

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond; //we have about 11.68 ft/s, we don't need to reduce it
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond; //360 degrees per second doesn't need to be reduced
        //These values could be tuned:
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.5;

        public static final double swerveModule1Offset = 70.75;
        public static final double swerveModule2Offset = -74.44;
        public static final double swerveModule3Offset = -30.61;
        public static final double swerveModule4Offset = -86.92;
    }

    //Checked and verified as of May 1st, 2022
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond; //2Pi
        //All the following Constants can be tuned:
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 2*Math.PI;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3.0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    //Checked and verified as of May 1st, 2022
    public static final class OIConstants {
        public static final int kDriverJoyID = 0;
        public static final int kDriverXL = 0;
        public static final int kDriverYL = 1;
        public static final int kDriverXR = 4;
        public static final int kDriverYR = 5;
        public static final double driverEXP = 7D/3D; //Exponentiate the joystick values to have finer control at low values

        public static final double kDeadband = 0.04; //Higher than average on the controller I'm using
    }
}
