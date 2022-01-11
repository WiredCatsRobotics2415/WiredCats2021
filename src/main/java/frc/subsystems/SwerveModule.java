package frc.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.util.Vector2D;
import frc.util.logging.MotorLogger;
import frc.util.logging.SwerveModuleLogger;
import frc.util.PWMAbsoluteEncoder;
import frc.util.pid.PIDFValue;
import frc.util.pid.PIDValue;
import frc.util.pid.TalonFxTunable;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class SwerveModule {

    private final TalonFX driveMotor, azimuthMotor; // motors

    private final PWMAbsoluteEncoder azimuthEncoder; // absolute encoder

    private final TalonFxTunable azimuthController;
    private final TalonFxTunable driveController;

    private MotorLogger logger;

    public final double positionX, positionY, radius;

    public static final double UNITS_TO_FT_DRIVE = 3.2 * 3.1415 / (12 * 7.0 * 2048.0);

    private double prevAzimuthSetpoint;
    private int turns;
    private boolean azimuthReversed;
    private Constants.SwerveModuleName name;
    private boolean testing;

    public SwerveModule(int driveMotorID, int azimuthMotorID, boolean azimuthRev, int azimuthEncoderChannel,
            double positionX, double positionY, PIDValue azimuthPidValues, double azimuthEncoderOffset,
            boolean azimuthEncoderReversed, PIDFValue drivePIDF) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.azimuthMotor = new TalonFX(azimuthMotorID);

        this.azimuthEncoder = new PWMAbsoluteEncoder(azimuthEncoderChannel, azimuthEncoderOffset,
                azimuthEncoderReversed);

        this.driveMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        this.azimuthMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        // Missing current limit
        this.driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);
        this.driveMotor.setSelectedSensorPosition(0, 0, Constants.kCanTimeoutMs);

        this.azimuthMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Constants.kCanTimeoutMs);
        this.azimuthMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);
        this.azimuthMotor.configSelectedFeedbackCoefficient(Constants.SWERVE_AZIMUTH_GEAR_RATIO, 0, Constants.kCanTimeoutMs);
        this.azimuthMotor.setSelectedSensorPosition(this.azimuthEncoder.getRotation2048(), 0, 10);

        this.driveMotor.setNeutralMode(Constants.DRIVE_BREAK_MODE);
        if(Constants.ZEROING) {
            this.azimuthMotor.setNeutralMode(NeutralMode.Coast);
        } else {
            this.azimuthMotor.setNeutralMode(NeutralMode.Brake);
        }
        this.driveMotor.setInverted(false);
        this.azimuthMotor.setInverted(false);

        this.azimuthMotor.configNominalOutputForward(0, Constants.kCanTimeoutMs);
        this.azimuthMotor.configNominalOutputReverse(0, Constants.kCanTimeoutMs);
        this.azimuthMotor.configPeakOutputForward(1, Constants.kCanTimeoutMs);
        this.azimuthMotor.configPeakOutputReverse(-1, Constants.kCanTimeoutMs);

        this.azimuthMotor.configClosedLoopPeakOutput(0, 0.8, Constants.kCanTimeoutMs);
        this.azimuthMotor.configNeutralDeadband(Constants.MOTORMIN, Constants.kCanTimeoutMs);

        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .1));
        this.azimuthMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, .1));

        this.azimuthMotor.config_kP(0, azimuthPidValues.getKP(), Constants.kCanTimeoutMs);
        this.azimuthMotor.config_kI(0, azimuthPidValues.getKI(), Constants.kCanTimeoutMs);
        this.azimuthMotor.config_kD(0, azimuthPidValues.getKD(), Constants.kCanTimeoutMs);

        // needs to be 0 if you are doing feedforward already
        this.driveMotor.config_kP(0, drivePIDF.getKP(), Constants.kCanTimeoutMs);
        this.driveMotor.config_kI(0, drivePIDF.getKI(), Constants.kCanTimeoutMs);
        this.driveMotor.config_kD(0, drivePIDF.getKD(), Constants.kCanTimeoutMs);
        this.driveMotor.config_kF(0, drivePIDF.getKF(), Constants.kCanTimeoutMs); 

        this.azimuthMotor.configAllowableClosedloopError(0, 4.0, Constants.kCanTimeoutMs);


        this.positionX = positionX;
        this.positionY = positionY;
        this.radius = Math.hypot(this.getPositionX() - RobotMap.CENTER_OF_MASS_X,
                this.getPositionY() - RobotMap.CENTER_OF_MASS_Y);

        this.azimuthReversed = false;

        this.azimuthController = new TalonFxTunable(this.azimuthMotor, azimuthPidValues, ControlMode.Position);
        this.driveController = new TalonFxTunable(this.driveMotor, drivePIDF, ControlMode.Velocity);

        this.zeroEncoder();

        this.prevAzimuthSetpoint = this.azimuthEncoder.getRotationDegrees();
        this.turns = 0;

        this.logger = null;
        this.name = null;
        this.testing = false;
    }

    public SwerveModule(int driveMotorID, int azimuthMotorID, boolean azimuthRev, int azimuthEncoderChannel,
            double positionX, double positionY, PIDValue pidValues, double azimuthEncoderOffset,
            boolean azimuthEncoderReversed, PIDFValue drivePIDF, boolean azimuthTuning, boolean logging, Constants.SwerveModuleName name) {
        this(driveMotorID, azimuthMotorID, azimuthRev, azimuthEncoderChannel, positionX, positionY, pidValues,
                azimuthEncoderOffset, azimuthEncoderReversed, drivePIDF);
        if (azimuthTuning) {
            this.azimuthController.enableTuning(name.toString());
        }
        if (logging) {
            this.logger = new MotorLogger(new SwerveModuleLogger(this, Constants.SWERVE_LOGGING_MODE));
        }
        this.name = name;
    }
/*
    public SwerveModule(boolean testing, int driveMotorID, int azimuthMotorID, boolean azimuthRev, int azimuthEncoderChannel,
    double positionX, double positionY, PIDValue pidValues, double azimuthEncoderOffset,
    boolean azimuthEncoderReversed, PIDFValue drivePIDF) {
        if(!testing) {
            this.driveMotor = new TalonFX(driveMotorID);
            this.azimuthMotor = new TalonFX(azimuthMotorID);

            this.azimuthEncoder = new PWMAbsoluteEncoder(azimuthEncoderChannel, azimuthEncoderOffset, azimuthReversed);

            this.driveMotor.configFactoryDefault(Constants.kCanTimeoutMs);
            this.driveMotor.configFactoryDefault(Constants.kCanTimeoutMs);
            // Missing current limit
            this.driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);
            this.azimuthMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);

            this.azimuthMotor.configSelectedFeedbackCoefficient(360.0/(2048*56.0/3.0), 0, Constants.kCanTimeoutMs);

            this.driveMotor.setNeutralMode(NeutralMode.Brake);
            this.azimuthMotor.setNeutralMode(NeutralMode.Brake);
            this.driveMotor.setInverted(false);
            this.azimuthMotor.setInverted(false);

            this.azimuthMotor.configNominalOutputForward(0, Constants.kCanTimeoutMs);
		    this.azimuthMotor.configNominalOutputReverse(0, Constants.kCanTimeoutMs);
		    this.azimuthMotor.configPeakOutputForward(1, Constants.kCanTimeoutMs);
            this.azimuthMotor.configPeakOutputReverse(-1, Constants.kCanTimeoutMs);

            this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .1));
            this.azimuthMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, .1));
        
            this.azimuthMotor.config_kP(0,pidValues.getKP(), Constants.kCanTimeoutMs);
            this.azimuthMotor.config_kI(0,pidValues.getKI(), Constants.kCanTimeoutMs);
            this.azimuthMotor.config_kD(0,pidValues.getKD(), Constants.kCanTimeoutMs);
        
            this.azimuthMotor.configAllowableClosedloopError(0, 2.0, Constants.kCanTimeoutMs);
        
            this.azimuthMotor.configIntegratedSensorOffset(this.azimuthEncoder.getRotationDegrees(), Constants.kCanTimeoutMs);

            this.positionX = positionX;
            this.positionY = positionY;
            this.radius = Math.hypot(this.getPositionX() - RobotMap.CENTER_OF_MASS_X,
                this.getPositionY() - RobotMap.CENTER_OF_MASS_Y);

            this.azimuthReversed = false;

            this.azimuthController = new TalonFxTunable(this.azimuthMotor, pidValues, ControlMode.Position);
            this.driveController = new TalonFxTunable(this.driveMotor, drivePIDF, ControlMode.Velocity);

            //this.azimuthController.setSetpoint(this.azimuthEncoder.getRotationDegrees());
            //setAzimuthControllerDegree(this.azimuthEncoder.getRotationDegrees());
            setAzimuthControllerRatio(this.azimuthEncoder.getRotationRaw(), 1024);
            this.prevAzimuthSetpoint = this.azimuthEncoder.getRotationRaw();
            this.turns = 0;
            this.testing = false;
        } else {
            this.testing = true;
            this.driveMotor = null;
            this.azimuthMotor = null;
            this.azimuthEncoder = new PWMAbsoluteEncoder(true, 0);
            this.positionX = positionX;
            this.positionY = positionY;
            this.radius = Math.hypot(this.getPositionX() - RobotMap.CENTER_OF_MASS_X,
                this.getPositionY() - RobotMap.CENTER_OF_MASS_Y);
            this.azimuthReversed = false;
            this.azimuthController = new TalonFxTunable(true, this.azimuthMotor, pidValues, ControlMode.Position);
            this.driveController = new TalonFxTunable(true, this.driveMotor, drivePIDF, ControlMode.Velocity);
            this.azimuthController.setSetpoint(0);
            this.prevAzimuthSetpoint = 0;
            this.turns = 0;
        }
    }
    */

    public void setVector(Vector2D vector) {
        if (vector.getLength() != 0) {
            setAngle(vector.getAngleDeg());
        } else {
            azimuthController.run();
        }
        if (azimuthReversed) {
            setPercentSpeed(vector.getLength() * -1);
        } else {
            setPercentSpeed(vector.getLength());
        }
    }

    public void setVelocityVectorWithFF(Vector2D vector, double ff) {
        if (Math.abs(vector.getLength()) > 0.1) {
            setAngle(vector.getAngleDeg());
            vector = vector.scale(0);
        } else {
            azimuthController.run();
        }
        vector.scale(UNITS_TO_FT_DRIVE);
        if (azimuthReversed) {
            setDriveVelocityWithFF(vector.getLength() * -1, -ff);
        } else {
            setDriveVelocityWithFF(vector.getLength(), ff);
        }
    }

    public void setAngle(double degrees) { 
        if (Math.abs(degrees - this.prevAzimuthSetpoint) > 90 && Math.abs(degrees - this.prevAzimuthSetpoint) < 270) {
            degrees = (degrees + 180) % 360;
            azimuthReversed = true;
        } else {
            azimuthReversed = false;
        }
        if (degrees > this.prevAzimuthSetpoint && Math.abs(degrees - this.prevAzimuthSetpoint) > 180) {
            turns--;
        } else if (degrees < this.prevAzimuthSetpoint && Math.abs(degrees - this.prevAzimuthSetpoint) > 180) {
            turns++;
        }
        //could be interesting - this would be an auto zero
        this.azimuthMotor.setSelectedSensorPosition(this.azimuthEncoder.getRotation2048());
        this.azimuthController.setSetpointDegrees(degrees + turns * 360.0);
        this.azimuthController.run();
        this.prevAzimuthSetpoint = degrees;
    }

    public void setPercentSpeed(double percent) {
        if(!testing) {
            this.driveMotor.set(TalonFXControlMode.PercentOutput, percent);
        }
    }

    public void setDriveVelocityWithFF(double velocity, double ff) {
        this.driveController.setSetpointWithFF(velocity, ff);
    }

    public void zeroEncoder() {
        this.azimuthMotor.set(ControlMode.PercentOutput, 0.0);
        this.azimuthMotor.setSelectedSensorPosition(this.azimuthEncoder.getRotation2048(), 0, 10);
        if (Math.abs(this.azimuthMotor.getSelectedSensorPosition() - this.azimuthEncoder.getRotation2048()) > 5) {
            //something is very wrong
            this.azimuthMotor.setSelectedSensorPosition(this.azimuthEncoder.getRotation2048(), 0, 30);
        } 
        //sometimes this doesn't work. e.g.: encoder says 1875, sensor says -1420, then after this it says a garbage negative that isn't 1875. not sure how to fix
        if(!Constants.ZEROING) {
            this.azimuthController.setSetpoint(this.azimuthEncoder.getRotation2048());
        }
        this.turns = 0;
    }

    public void zeroDriveEncoder() {
        this.driveMotor.setSelectedSensorPosition(0.0, 0, Constants.kCanTimeoutMs);
    }

    public void setAngleSimple(double degrees) {
        azimuthController.setSetpointDegrees(degrees);
        azimuthController.run();
    }

    public void saveLog() {
        if (this.logger != null) {
            this.logger.saveDataToCSV(this.name + "logging.csv");
        }
    }

    public double getDrivePosition() {
        return this.driveMotor.getSelectedSensorPosition(0);
    }

    public double getDriveVoltage() {
        return this.driveMotor.getMotorOutputVoltage();
    }

    public double getDriveVelocity() {
        return this.driveMotor.getSelectedSensorVelocity(0);
    }

    public double getDriveCurrent() {
        return this.driveMotor.getStatorCurrent();
    }

    public double getAzimuthAngle() {
        return this.azimuthEncoder.getRotationDegrees();
    }

    public double getAzimuthVoltage() {
        return this.azimuthMotor.getMotorOutputVoltage();
    }

    public double getAzimuthCurrent() {
        return this.azimuthMotor.getStatorCurrent();
    }

    public double getPercentOutput() {
        return this.driveMotor.getMotorOutputPercent();
    }

    public double getRadius() {
        return this.radius;
    }

    public double getPositionX() {
        return positionX;
    }

    public double getPositionY() {
        return positionY;
    }

    public void log() {
        if (this.logger == null)
            return;
        this.logger.run();
    }

    public void printCurrent() {
        System.out.println(this.name + " " + this.driveMotor.getStatorCurrent());
    }

    public void printAzimuthTalonEncoderValue() {
        System.out.println("TalonFx " + this.name.toString() + " Value:"
                + this.azimuthMotor.getSelectedSensorPosition(0) % 360 + "degrees");
    }

    public void printAzimuthEncoderValue() {
        System.out.println("MA3 " + this.name.toString() + " Value:"
                + this.azimuthEncoder.getRotationDegrees() + "degrees");
    }
    
    public double getAzimuthSetpoint() {
        return this.azimuthController.getSetpoint();
    }

}
