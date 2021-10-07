package frc.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.PWMAbsoluteEncoder;
import frc.util.pid.TalonFxTunable;

public class Turret {
    private TalonFX rightShooterTalon, leftShooterTalon;
    private TalonFxTunable shooterController;

    private TalonFX turretMotor;
    private TalonFxTunable turretController;

    private PWMAbsoluteEncoder turnEncoder;

    private CANSparkMax hoodMotor;
    private PWMAbsoluteEncoder hoodEncoder;

    private double hoodSetpoint;
    private boolean hoodStuck;
    private final double STUCK_POINT = 0.0;



    public Turret() {
        // create motors
        this.leftShooterTalon = new TalonFX(RobotMap.LEFT_TURRET_MOTOR);
        this.rightShooterTalon = new TalonFX(RobotMap.RIGHT_TURRET_MOTOR);
        this.turretMotor = new TalonFX(RobotMap.TURN_TURRET_MOTOR);

        
        this.turnEncoder = new PWMAbsoluteEncoder(RobotMap.TURN_TURRET_ENCODER, Constants.TURN_TURRET_ENCODER_OFFSET, RobotMap.TURN_TURRET_ENCODER_REV);
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
        this.turretMotor.configSelectedFeedbackCoefficient(360.0/(2048.0*1.0/90.0), 0,
                Constants.kCanTimeoutMs);
        // set brake modes
        this.leftShooterTalon.setNeutralMode(NeutralMode.Coast);
        this.rightShooterTalon.setNeutralMode(NeutralMode.Coast);
        this.turretMotor.setNeutralMode(NeutralMode.Brake);
        // set directions (because the motors on the shooter are opposite, one probably
        // needs to be inverted but check to be sure)
        // (don't drive both at the same time the 1st run)
        this.leftShooterTalon.setInverted(true);
        this.rightShooterTalon.setInverted(false);
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

        this.shooterController = new TalonFxTunable(this.rightShooterTalon, Constants.SHOOTER_PIDF, ControlMode.Velocity);
        this.leftShooterTalon.set(ControlMode.Follower, this.rightShooterTalon.getDeviceID());
        this.turretController = new TalonFxTunable(this.turretMotor, Constants.TURRET_PID, ControlMode.Position);

        this.shooterController.setSetpoint(0);
        this.turretController.setSetpoint(turnEncoder.getRotationDegrees()); // should be initial encoder value

        this.hoodMotor = new CANSparkMax(RobotMap.HOOD_TURRET_MOTOR, CANSparkMax.MotorType.kBrushless);
        this.hoodMotor.restoreFactoryDefaults();
        this.hoodMotor.setIdleMode(IdleMode.kBrake);

        this.hoodEncoder = new PWMAbsoluteEncoder(RobotMap.HOOD_TURRET_ENCODER, Constants.HOOD_TURRET_ENCODER_OFFSET, RobotMap.HOOD_TURRET_ENCODER_REV);

        this.hoodMotor.setInverted(true);
        this.hoodMotor.getEncoder().setPositionConversionFactor(3000);
        this.hoodMotor.getEncoder().setPosition(hoodEncoder.getRotationDegrees());
        if (this.hoodEncoder.getRotationDegrees() < STUCK_POINT) hoodStuck = true;

        CANPIDController hoodPID = this.hoodMotor.getPIDController();
        hoodPID.setP(Constants.HOOD_PID.getKP());
        hoodPID.setI(Constants.HOOD_PID.getKI());
        hoodPID.setD(Constants.HOOD_PID.getKD());
        hoodPID.setReference(hoodEncoder.getRotationDegrees(), ControlType.kPosition);

    }

    public void setShooterSpeed(double speed) {
        System.out.println("Set to " + speed);
        shooterController.setSetpoint(speed);
    }

    public double getShooterSpeed() {
        return rightShooterTalon.getSelectedSensorVelocity(0);
    }

    public void stopShooter() {
        shooterController.setSetpoint(0);
    }

    public CANPIDController getHoodPID() {
        return hoodMotor.getPIDController();
    }

    public void setTurretAngle(double degrees) {
        if (degrees <= 180 || degrees >= -180) {
            this.turretController.setSetpoint(degrees);
            this.turretController.run();
        }
    }

    public void run() {
        this.shooterController.run();
        this.turretController.run();
    }

    public void zeroTurret() {
        this.turretMotor.set(ControlMode.PercentOutput, 0);
        ErrorCode error = this.turretMotor.setSelectedSensorPosition(this.turnEncoder.getRotationDegrees(), 0, Constants.kCanTimeoutMs);
        if (!error.equals(ErrorCode.OK)) {
            System.out.println("failed zero");
            error = this.turretMotor.setSelectedSensorPosition(this.turnEncoder.getRotationDegrees(), 0,
                    Constants.kCanTimeoutMs);
        }
        if(!Constants.ZEROING) { //might want to have separate zeroing constant for turret
            this.turretController.setSetpoint(this.turnEncoder.getRotationDegrees());
        }
    }

    public double getTurretAngle() {
        return this.turretController.getSetpoint(); 
    }

    public void setHoodAngle(double angle) {
        if (angle > STUCK_POINT && hoodStuck) {
            while (this.hoodEncoder.getRotationDegrees() < STUCK_POINT) this.hoodMotor.set(1);
            this.hoodMotor.set(0);
            this.hoodMotor.getEncoder().setPosition(hoodEncoder.getRotationDegrees());
            hoodStuck = false;
        }
        if (!hoodStuck) {
            hoodSetpoint = angle;
            this.hoodMotor.getPIDController().setReference(angle, ControlType.kPosition);
            if (angle < STUCK_POINT) hoodStuck = true;
        }
    }

    public void stopHood() {
        this.hoodMotor.stopMotor();
    }
    
    public double getHoodAngle() {
        return this.hoodMotor.getEncoder().getPosition();
    }

    public double getHoodSetpoint() {
        return this.hoodSetpoint;
    }


    public TalonFX getShooter() {
        return this.rightShooterTalon;
    }

    public void zeroHood() {
        this.hoodMotor.stopMotor();
        this.hoodMotor.getEncoder().setPosition(this.hoodEncoder.getRotationDegrees());
        if (!Constants.ZEROING) {
            this.hoodMotor.getPIDController().setReference(this.hoodEncoder.getRotationDegrees(), ControlType.kPosition);
        }
    }

    public void printEncoderValues() {
        System.out.println("Turret turn: " + this.turnEncoder.getRotationDegrees());
        System.out.println("Turret hood: " + this.hoodEncoder.getRotationDegrees());
    }
}
