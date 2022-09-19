package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import static frc.robot.Constants.kTalonConfigTimeout;

public class SwerveModule {

    private final WPI_TalonFX driveTalon;
    private final WPI_TalonSRX azimuthTalon;
/*
    private final double driveEncoderPos;
    private final double driveEncoderVel;
    private final double azimuthEncoderPos;
    private final double azimuthEncoderVel;

    private final double absoluteEncoderPos;*/
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveTalonId, int azimuthTalonId, boolean driveTalonReversed, boolean azimuthTalonReversed,
            double absoluteEncoderOffset, boolean absoluteEncoderReversed, TalonSRXConfiguration azimuthConfig, TalonFXConfiguration driveConfig) {
                
            azimuthTalon = new WPI_TalonSRX(azimuthTalonId);
            azimuthTalon.configFactoryDefault(kTalonConfigTimeout);
            azimuthTalon.configAllSettings(ModuleConstants.getAzimuthTalonConfig(), kTalonConfigTimeout);
            azimuthTalon.enableCurrentLimit(true);
            azimuthTalon.enableVoltageCompensation(true);
            azimuthTalon.configFeedbackNotContinuous(true, kTalonConfigTimeout);
            azimuthTalon.setNeutralMode(NeutralMode.Coast);
            

            driveTalon = new WPI_TalonFX(driveTalonId, Constants.kCanivoreName);
            driveTalon.configFactoryDefault(kTalonConfigTimeout);
            driveTalon.configAllSettings(ModuleConstants.getDriveTalonConfig(), kTalonConfigTimeout);
            driveTalon.enableVoltageCompensation(true);
            driveTalon.setNeutralMode(NeutralMode.Coast);
            driveTalon.setInverted(TalonFXInvertType.CounterClockwise);
            

            this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
            this.absoluteEncoderReversed = absoluteEncoderReversed;

            resetEncoders();
        }

//Return drive position in meters
    public double getDrivePosition() {
        return driveTalon.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderMeterPerTick;
    }
//Return azimuth position in radians
    public double getAzimuthPosition() {
        return azimuthTalon.getSelectedSensorPosition(0) * ModuleConstants.kAzimuthEncoderRadPerTick;
    }
//Return drive velocity in meters per second
    public double getDriveVelocity() {
        return driveTalon.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderMeterPerSec;
    }
//Return azimuth velocity in radians per second
    public double getAzimuthVelocity() {
        return azimuthTalon.getSelectedSensorVelocity(0) * ModuleConstants.kAzimuthEncoderRadPerSec;
    }
//Return raw absolute azimuth position in radians
    public double getAbsoluteEncoderRadRaw() {
        double angle = azimuthTalon.getSelectedSensorPosition(1) * ModuleConstants.kAzimuthEncoderRadPerTick;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
//Return absolute azimuth position in ticks
    public double getAbsoluteEncoderPos() {
        double angle = azimuthTalon.getSelectedSensorPosition(1);
        angle -= absoluteEncoderOffsetRad / ModuleConstants.kAzimuthEncoderRadPerTick;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveTalon.setSelectedSensorPosition(0);
        azimuthTalon.setSelectedSensorPosition(getAbsoluteEncoderPos() + ModuleConstants.kAzimuthEncoderTicksPerRev/2 , 0, kTalonConfigTimeout);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAzimuthPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

//Convert module state angle from a range of (-pi, pi) to (0, 2pi)
        double steerAngle = state.angle.getRadians();
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double currentAngleRadians = azimuthTalon.getSelectedSensorPosition() * ModuleConstants.kAzimuthEncoderRadPerTick;
        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }
        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
        double adjustedReferenceAngleRadians = steerAngle + currentAngleRadians - currentAngleRadiansMod;
        if (steerAngle - currentAngleRadiansMod > Math.PI) {
             adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (steerAngle - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        driveTalon.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        azimuthTalon.set(ControlMode.MotionMagic,  adjustedReferenceAngleRadians / ModuleConstants.kAzimuthEncoderRadPerTick);
    }

    public void stop() {
        driveTalon.set(0);
        azimuthTalon.set(0);
    }
}
