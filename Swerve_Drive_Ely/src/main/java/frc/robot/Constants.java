package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public final class Constants {

    public static String kCanivoreName = "falcons-canivore";
    public static final int kTalonConfigTimeout = 10; // ms


    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
        public static final double kDriveMotorGearRatio = 1 / 5.25;
        public static final double kAzimuthEncoderGearRatio = 1 / 1;
        public static final double kDriveEncoderTicksPerRev = 2048;
        public static final double kAzimuthEncoderTicksPerRev = 4096;
        public static final double kCTREvelUnits2TicksperSec = 0.1; // 1/10 seconds per 100ms
        public static final double kDriveEncoderMeterPerTick =  (1/kDriveEncoderTicksPerRev) * kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI;
        public static final double kAzimuthEncoderRadPerTick = (2 * Math.PI) / kAzimuthEncoderTicksPerRev / kAzimuthEncoderGearRatio;
        public static final double kDriveEncoderMeterPerSec = kDriveEncoderMeterPerTick / kCTREvelUnits2TicksperSec;
        public static final double kAzimuthEncoderRadPerSec = kAzimuthEncoderRadPerTick / kCTREvelUnits2TicksperSec;
        public static TalonFXConfiguration driveConfig;
        public static TalonSRXConfiguration azimuthConfig;

        public static TalonSRXConfiguration getAzimuthTalonConfig() {
            TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
        
            azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
            azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Absolute;
        
            azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
            azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

            azimuthConfig.continuousCurrentLimit = 10;
            azimuthConfig.peakCurrentDuration = 0;
            azimuthConfig.peakCurrentLimit = 0;
            azimuthConfig.slot0.kP = 7.0;
            azimuthConfig.slot0.kI = 0.0;
            azimuthConfig.slot0.kD = 100.0;
            azimuthConfig.slot0.kF = 0.0;
            azimuthConfig.slot0.integralZone = 0;
            azimuthConfig.slot0.allowableClosedloopError = 0;
            azimuthConfig.slot0.maxIntegralAccumulator = 0;
            azimuthConfig.motionCruiseVelocity = 800;
            azimuthConfig.motionAcceleration = 10_000;
            azimuthConfig.velocityMeasurementWindow = 64;
            azimuthConfig.voltageCompSaturation = 12;
            return azimuthConfig;
        }
        
        public static TalonFXConfiguration getDriveTalonConfig() {
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();

            driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
            driveConfig.supplyCurrLimit.currentLimit = 0.04;
            driveConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
            driveConfig.supplyCurrLimit.triggerThresholdTime = 40;
            driveConfig.supplyCurrLimit.enable = true;
            driveConfig.slot0.kP = 0.045;
            driveConfig.slot0.kI = 0.0005;
            driveConfig.slot0.kD = 0.000;
            driveConfig.slot0.kF = 0.047;
            driveConfig.slot0.integralZone = 500;
            driveConfig.slot0.maxIntegralAccumulator = 75_000;
            driveConfig.slot0.allowableClosedloopError = 0;
            driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
            driveConfig.velocityMeasurementWindow = 64;
            driveConfig.voltageCompSaturation = 12;
            return driveConfig;
        }
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(19.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));


        public static final int kFrontLeftDriveMotorPort = 20;
        public static final int kBackLeftDriveMotorPort = 22;
        public static final int kFrontRightDriveMotorPort = 21;
        public static final int kBackRightDriveMotorPort = 23;

        public static final int kFrontLeftAzimuthMotorPort = 10;
        public static final int kBackLeftAzimuthMotorPort = 12;
        public static final int kFrontRightAzimuthMotorPort = 11;
        public static final int kBackRightAzimuthMotorPort = 13;

        public static final boolean kFrontLeftAzimuthEncoderReversed = false;
        public static final boolean kBackLeftAzimuthEncoderReversed = false;
        public static final boolean kFrontRightAzimuthEncoderReversed = false;
        public static final boolean kBackRightAzimuthEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 3.558;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.225;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 4.1678;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 4.453; 

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.84632;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond /4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond /4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 6;

        public static final double kDeadband = 0.075;
    }
}
