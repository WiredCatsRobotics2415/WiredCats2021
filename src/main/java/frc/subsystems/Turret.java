package frc.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.PWMAbsoluteEncoder;
import frc.util.pid.TalonFxTunable;

public class Turret {
    private TalonFX rightShooterTalon, leftShooterTalon;
    private TalonFxTunable shooterController;

    private TalonFX turretMotor;
    private TalonFxTunable turretController;

    private PWMAbsoluteEncoder encoder;

    private boolean turnReversed;
    private double prevTurnSetpoint;
    private int turns;

    public Turret() {
        // create motors
        this.leftShooterTalon = new TalonFX(RobotMap.LEFT_TURRET_MOTOR);
        this.rightShooterTalon = new TalonFX(RobotMap.RIGHT_TURRET_MOTOR);
        this.turretMotor = new TalonFX(RobotMap.TURN_TURRET_MOTOR);

        this.encoder = new PWMAbsoluteEncoder(RobotMap.TURRET_ENCODER, Constants.TURRET_ENCODER_OFFSET, RobotMap.TURRET_ENCODER_REV);
        // declare settings for motors
        this.leftShooterTalon.configFactoryDefault(Constants.kCanTimeoutMs);
        this.rightShooterTalon.configFactoryDefault(Constants.kCanTimeoutMs);
        this.turretMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        // set encoder start configuration (start at 0)
        this.leftShooterTalon.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
                Constants.kCanTimeoutMs);
        this.rightShooterTalon.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
                Constants.kCanTimeoutMs);
        this.turretMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
                Constants.kCanTimeoutMs);
        // set feedback device to be internal encoder
        this.leftShooterTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);
        this.rightShooterTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);
        this.turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);
        // set ratio of turret motor turns to turret turns
        this.turretMotor.configSelectedFeedbackCoefficient(1/* ratio of motor turns to turret turns */, 0,
                Constants.kCanTimeoutMs);
        // set brake modes
        this.leftShooterTalon.setNeutralMode(NeutralMode.Coast);
        this.rightShooterTalon.setNeutralMode(NeutralMode.Coast);
        this.turretMotor.setNeutralMode(NeutralMode.Brake);
        // set directions (because the motors on the shooter are opposite, one probably
        // needs to be inverted but check to be sure)
        // (don't drive both at the same time the 1st run)
        this.leftShooterTalon.setInverted(false);
        this.rightShooterTalon.setInverted(true);
        this.turretMotor.setInverted(false);
        // curent limits (you should play with the settings, I choose these kinda
        // random)
        this.leftShooterTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .1));
        this.rightShooterTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .1));
        this.turretMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, .1));
        // max output by closed loop control
        this.turretMotor.configClosedLoopPeakOutput(0, 0.8, Constants.kCanTimeoutMs);
        // deadband for closed loop error
        this.turretMotor.configAllowableClosedloopError(0, 3.0, Constants.kCanTimeoutMs);
        // DeadBand
        this.leftShooterTalon.configNeutralDeadband(Constants.MOTORMIN, Constants.kCanTimeoutMs);
        this.rightShooterTalon.configNeutralDeadband(Constants.MOTORMIN, Constants.kCanTimeoutMs);
        this.turretMotor.configNeutralDeadband(Constants.MOTORMIN, Constants.kCanTimeoutMs);

        this.shooterController = new TalonFxTunable(this.rightShooterTalon, null, ControlMode.Velocity);
        this.leftShooterTalon.set(ControlMode.Follower, this.rightShooterTalon.getDeviceID());
        this.turretController = new TalonFxTunable(this.turretMotor, null, ControlMode.Position);

        this.shooterController.setSetpoint(0);
        this.turretController.setSetpoint(encoder.getRotationDegrees()); // should be initial encoder value

        turnReversed = false;
        prevTurnSetpoint = this.turretController.getSetpoint();
        turns = 0;
    }

    public void setShooterSpeed(double speed) {
        shooterController.setSetpoint(speed);
    }

    public void stopShooter() {
        shooterController.setSetpoint(0);
    }

    public void setAngle(double degrees) {
        if (Math.abs(degrees - this.prevTurnSetpoint) > 90 && Math.abs(degrees - this.prevTurnSetpoint) < 270) {
            degrees = (degrees + 180) % 360;
            turnReversed = true;
        } else {
            turnReversed = false;
        }
        if (degrees > this.prevTurnSetpoint && Math.abs(degrees - this.prevTurnSetpoint) > 180) {
            turns--;
        } else if (degrees < this.prevTurnSetpoint && Math.abs(degrees - this.prevTurnSetpoint) > 180) {
            turns++;
        }
        this.turretController.setSetpoint(degrees + turns * 360.0);
        this.prevTurnSetpoint = degrees;
    }

    public void run() {
        this.shooterController.run();
        this.turretController.run();
    }

    public void zeroEncoder() {
        this.turretMotor.set(ControlMode.PercentOutput, 0);
        ErrorCode error = this.turretMotor.setSelectedSensorPosition(this.encoder.getRotationDegrees(), 0, Constants.kCanTimeoutMs);
        if (!error.equals(ErrorCode.OK)) {
            System.out.println("failed zero");
            error = this.turretMotor.setSelectedSensorPosition(this.encoder.getRotationDegrees(), 0,
                    Constants.kCanTimeoutMs);
        }
        if(!Constants.ZEROING) {
            this.turretController.setSetpoint(this.encoder.getRotationDegrees());
        }
    }

}
