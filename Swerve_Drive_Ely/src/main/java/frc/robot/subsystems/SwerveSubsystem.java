package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftAzimuthMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftAzimuthEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            ModuleConstants.azimuthConfig,
            ModuleConstants.driveConfig);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightAzimuthMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightAzimuthEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            ModuleConstants.azimuthConfig,
            ModuleConstants.driveConfig);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftAzimuthMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftAzimuthEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            ModuleConstants.azimuthConfig,
            ModuleConstants.driveConfig);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightAzimuthMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightAzimuthEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            ModuleConstants.azimuthConfig,
            ModuleConstants.driveConfig);

    private final AHRS gyro = new AHRS(I2C.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(6000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
//Robot State
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
//Module Absolute Angle Raw
        SmartDashboard.putNumber("Front Left Azimuth Absolute", frontLeft.getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("Front Right Azimuth Absolute", frontRight.getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("Back Left Azimuth Absolute", backLeft.getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("Back Right Azimuth Absolute", backRight.getAbsoluteEncoderRadRaw());
//Module Angle
        SmartDashboard.putNumber("Front Left Azimuth", frontLeft.getAzimuthPosition());
        SmartDashboard.putNumber("Front Right Azimuth", frontRight.getAzimuthPosition());
        SmartDashboard.putNumber("Back Left Azimuth", backLeft.getAzimuthPosition());
        SmartDashboard.putNumber("Back Right Azimuth", backRight.getAzimuthPosition());
//Gyro Angle
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
//Wheel Speed
        SmartDashboard.putNumber("Front Left Drive m/s", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("Front Right Drive m/s", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("Back Left Drive m/s", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("Back Right Drive m/s", backRight.getDriveVelocity());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
        //SmartDashboard.putString("Back Right Desired Angle", desiredStates[3].angle.toString());
    }
}
