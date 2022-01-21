package frc.subsystems.newswerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
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

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.KS, Constants.KV, Constants.KA);

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
        driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
                Constants.kCanTimeoutMs);
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

    public static SwerveModuleState optimize(SwerveModuleState state, Rotation2d currentAngle) {
        double speed = state.speedMetersPerSecond;
        double targetAngle = state.angle.getDegrees();
        if (Math.floor(currentAngle.getDegrees() / 360.0) != 0) {
            // always positive modulus function = (a % b + b) % b
            // taking the angle in [0, 360) then adding the number of full rotations * 360
            targetAngle = ((state.angle.getDegrees() % 360 + 360) % 360)
                    + Math.floor(currentAngle.getDegrees() / 360.0) * 360.0;
        }
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            speed *= -1;
            if (delta > 0) {
                targetAngle -= 180;
            } else {
                targetAngle += 180;
            }
        }
        return new SwerveModuleState(speed, Rotation2d.fromDegrees(targetAngle));
    }

    public void setNewState(SwerveModuleState newState, boolean openLoop) {
        newState = optimize(newState, getState().angle);
        if (openLoop)
            driveMotor.set(ControlMode.PercentOutput, newState.speedMetersPerSecond / Constants.MAX_SWERVE_SPEED);
        else
            driveMotor.set(ControlMode.Velocity, Constants.MPSToFalcon(newState.speedMetersPerSecond),
                    DemandType.ArbitraryFeedForward, feedforward.calculate(newState.speedMetersPerSecond));
        // Module jitter deadband
        double angle = (Math.abs(newState.speedMetersPerSecond) <= (Constants.MAX_SWERVE_SPEED * 0.1)) ? lastAngle
                : newState.angle.getDegrees();
        azimuthMotor.set(ControlMode.Position, Constants.degreesToFalcon(angle));
        lastAngle = angle;
    }

    private void configAzimuthMotor(PIDValue pid) {
        azimuthMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        azimuthMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCanTimeoutMs);
        azimuthMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
                Constants.kCanTimeoutMs);
        azimuthMotor.setSelectedSensorPosition(this.azimuthEncoder.getRotation2048(), 0, Constants.kCanTimeoutMs);
        if (Constants.ZEROING)
            azimuthMotor.setNeutralMode(NeutralMode.Coast);
        else
            azimuthMotor.setNeutralMode(NeutralMode.Brake);
        azimuthMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1));
        azimuthMotor.config_kP(0, pid.getKP(), Constants.kCanTimeoutMs);
        azimuthMotor.config_kI(0, pid.getKI(), Constants.kCanTimeoutMs);
        azimuthMotor.config_kD(0, pid.getKD(), Constants.kCanTimeoutMs);
    }

    public void syncAzimuth() {
        azimuthMotor.setSelectedSensorPosition(azimuthEncoder.getRotation2048(), 0, Constants.kCanTimeoutMs);
    }

    public SwerveModuleState getState() {
        double velocity = Constants.falconToMPS(driveMotor.getSelectedSensorVelocity(0));
        Rotation2d angle = Rotation2d.fromDegrees(Constants.falconToDegrees(azimuthMotor.getSelectedSensorPosition(0)));
        return new SwerveModuleState(velocity, angle);
    }

    public Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(azimuthEncoder.getRotationDegrees());
    }

    public void printTalonEncoder() {
        System.out.println("TalonFx " + this.name.toString() + " Value:"
                + this.azimuthMotor.getSelectedSensorPosition(0) % 360 + "degrees");
    }

    public void printAzimuthEncoder() {
        System.out.println("MA3 " + this.name.toString() + " Value:"
                + this.azimuthEncoder.getRotationDegrees() + "degrees");
    }

}
