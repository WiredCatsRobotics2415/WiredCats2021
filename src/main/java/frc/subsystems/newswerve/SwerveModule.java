package frc.subsystems.newswerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.PWMAbsoluteEncoder;
import frc.util.pid.PIDFValue;
import frc.util.pid.PIDValue;

public class SwerveModule {

    private TalonFX driveMotor;
    private TalonFX azimuthMotor;
    private PWMAbsoluteEncoder azimuthEncoder;
    private double lastAngle;
    private Constants.SwerveModuleName name;

    public SwerveModule(Constants.SwerveModuleName name) {
        this.driveMotor = new TalonFX(RobotMap.DRIVE_PORTS[name.ordinal()]);
        this.azimuthMotor = new TalonFX(RobotMap.AZIMUTH_ENCODER_PORTS[name.ordinal()]);
        this.azimuthEncoder = new PWMAbsoluteEncoder(RobotMap.AZIMUTH_ENCODER_PORTS[name.ordinal()]);
        configDriveMotor(Constants.DRIVE_PIDF_VALUES[name.ordinal()]);
        configAzimuthMotor(Constants.AZIMUTH_PID_VALUES[name.ordinal()]);
        azimuthEncoder.setOffset(Constants.ENCODER_OFFSETS[name.ordinal()]);
        this.lastAngle = azimuthEncoder.getRotationDegrees();
    }

    private void configDriveMotor(PIDFValue pidf) {
        driveMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCanTimeoutMs);
        driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Constants.kCanTimeoutMs);
        driveMotor.setSelectedSensorPosition(0, 0, Constants.kCanTimeoutMs);
        driveMotor.setNeutralMode(Constants.DRIVE_BREAK_MODE);
        driveMotor.setInverted(false);
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1));
        driveMotor.configOpenloopRamp(0.25, Constants.kCanTimeoutMs);
        driveMotor.configClosedloopRamp(0.0, Constants.kCanTimeoutMs);
        driveMotor.config_kP(0, pidf.getKP(), Constants.kCanTimeoutMs);
        driveMotor.config_kI(0, pidf.getKI(), Constants.kCanTimeoutMs);
        driveMotor.config_kD(0, pidf.getKD(), Constants.kCanTimeoutMs);
        driveMotor.config_kF(0, pidf.getKF(), Constants.kCanTimeoutMs); 
    }

    private void configAzimuthMotor(PIDValue pid) {
        azimuthMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        azimuthMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCanTimeoutMs);
        azimuthMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Constants.kCanTimeoutMs);
        azimuthMotor.setSelectedSensorPosition(this.azimuthEncoder.getRotation2048(), 0, Constants.kCanTimeoutMs);
        if(Constants.ZEROING) azimuthMotor.setNeutralMode(NeutralMode.Coast);
        else azimuthMotor.setNeutralMode(NeutralMode.Brake);
        azimuthMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1));
        azimuthMotor.config_kP(0, pid.getKP(), Constants.kCanTimeoutMs);
        azimuthMotor.config_kI(0, pid.getKI(), Constants.kCanTimeoutMs);
        azimuthMotor.config_kD(0, pid.getKD(), Constants.kCanTimeoutMs);
    }

    public void syncAzimuth() {
        azimuthMotor.setSelectedSensorPosition(azimuthEncoder.getRotation2048(), 0, Constants.kCanTimeoutMs);
    }

    public SwerveModuleState getState() {
    }

    public Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(azimuthEncoder.getRotationDegrees());
    }
    
}
