package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {

    SwerveModule[] mSwerveMods = {
            new SwerveModule(
                    DriveConstants.kFrontLeftDriveMotorPort,
                    DriveConstants.kFrontLeftAzimuthMotorPort,
                    DriveConstants.kFrontLeftDriveEncoderReversed,
                    DriveConstants.kFrontLeftAzimuthEncoderReversed,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
                    ModuleConstants.azimuthConfig,
                    ModuleConstants.driveConfig),
            new SwerveModule(
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightAzimuthMotorPort,
                    DriveConstants.kFrontRightDriveEncoderReversed,
                    DriveConstants.kFrontRightAzimuthEncoderReversed,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
                    ModuleConstants.azimuthConfig,
                    ModuleConstants.driveConfig),
            new SwerveModule(
                    DriveConstants.kBackLeftDriveMotorPort,
                    DriveConstants.kBackLeftAzimuthMotorPort,
                    DriveConstants.kBackLeftDriveEncoderReversed,
                    DriveConstants.kBackLeftAzimuthEncoderReversed,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
                    ModuleConstants.azimuthConfig,
                    ModuleConstants.driveConfig),
            new SwerveModule(
                    DriveConstants.kBackRightDriveMotorPort,
                    DriveConstants.kBackRightAzimuthMotorPort,
                    DriveConstants.kBackRightDriveEncoderReversed,
                    DriveConstants.kBackRightAzimuthEncoderReversed,
                    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
                    ModuleConstants.azimuthConfig,
                    ModuleConstants.driveConfig) };

    SwerveModulePosition[] mPositions = new SwerveModulePosition[4];

    private final AHRS gyro = new AHRS(I2C.Port.kMXP);
    private final SwerveDriveOdometry odometer;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(6000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getPositions() );
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public SwerveModulePosition[] getPositions() {
        // SwerveModulePosition[] positions = new SwerveModulePosition[4];
        mPositions[0] = mSwerveMods[0].getPosition();
        mPositions[1] = mSwerveMods[1].getPosition();
        mPositions[2] = mSwerveMods[2].getPosition();
        mPositions[3] = mSwerveMods[3].getPosition();
        return mPositions;
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
        odometer.resetPosition(getRotation2d(), getPositions(), pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getPositions());
        // Robot State
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        // Module Absolute Angle Raw
        SmartDashboard.putNumber("Front Left Azimuth Absolute", mSwerveMods[0].getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("Front Right Azimuth Absolute", mSwerveMods[1].getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("Back Left Azimuth Absolute", mSwerveMods[2].getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("Back Right Azimuth Absolute", mSwerveMods[3].getAbsoluteEncoderRadRaw());
        // Module Angle
        SmartDashboard.putNumber("Front Left Azimuth", mSwerveMods[0].getAzimuthPosition());
        SmartDashboard.putNumber("Front Right Azimuth", mSwerveMods[1].getAzimuthPosition());
        SmartDashboard.putNumber("Back Left Azimuth", mSwerveMods[2].getAzimuthPosition());
        SmartDashboard.putNumber("Back Right Azimuth", mSwerveMods[3].getAzimuthPosition());
        // Gyro Angle
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        // Wheel Speed
        SmartDashboard.putNumber("Front Left Drive m/s", mSwerveMods[0].getDriveVelocity());
        SmartDashboard.putNumber("Front Right Drive m/s", mSwerveMods[1].getDriveVelocity());
        SmartDashboard.putNumber("Back Left Drive m/s", mSwerveMods[2].getDriveVelocity());
        SmartDashboard.putNumber("Back Right Drive m/s", mSwerveMods[3].getDriveVelocity());
    }

    public void stopModules() {
        mSwerveMods[0].stop();
        mSwerveMods[1].stop();
        mSwerveMods[2].stop();
        mSwerveMods[3].stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        mSwerveMods[0].setDesiredState(desiredStates[0]);
        mSwerveMods[1].setDesiredState(desiredStates[1]);
        mSwerveMods[2].setDesiredState(desiredStates[2]);
        mSwerveMods[3].setDesiredState(desiredStates[3]);
        // SmartDashboard.putString("Back Right Desired Angle",
        // desiredStates[3].angle.toString());
    }
}
