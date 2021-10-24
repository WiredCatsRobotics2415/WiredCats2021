package frc.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.util.Vector2D;

public class SwerveDrive {
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final AHRS navX;

    private final double maxModuleRadius;
    // private double prevAngle;
    // private long prevTime;

    public static final double TURNING_KP = 0.015, TURNING_KD = 0.0, MAX_ADJUSTMENT = 0.3;

    private SwerveSpecialMode specialMode;

    public enum SwerveSpecialMode {
        RIGHT_TURNING, LEFT_TURNING, NORMAL;
    }

    public SwerveDrive() {
        this(false, false);
    }

    public SwerveDrive(boolean tuning, boolean logging) {
        this.frontLeftModule = new SwerveModule(RobotMap.FRONT_LEFT_SWERVE_DRIVE, RobotMap.FRONT_LEFT_SWERVE_AZIMUTH,
                RobotMap.FRONT_LEFT_SWERVE_AZIMUTH_REV, RobotMap.FRONT_LEFT_AZIMUTH_ENCODER,
                RobotMap.FRONT_LEFT_MODULE_X, RobotMap.FRONT_LEFT_MODULE_Y, Constants.FRONT_LEFT_AZIMUTH_PID,
                Constants.FRONT_LEFT_AZIMUTH_ENCODER_OFFSET, RobotMap.FRONT_LEFT_AZIMUTH_ENCODER_REV,
                Constants.FRONT_LEFT_VEL_DRIVE_PIDF, tuning, logging, Constants.SwerveModuleName.FRONT_LEFT);
        this.frontRightModule = new SwerveModule(RobotMap.FRONT_RIGHT_SWERVE_DRIVE, RobotMap.FRONT_RIGHT_SWERVE_AZIMUTH,
                RobotMap.FRONT_RIGHT_SWERVE_AZIMUTH_REV, RobotMap.FRONT_RIGHT_AZIMUTH_ENCODER,
                RobotMap.FRONT_RIGHT_MODULE_X, RobotMap.FRONT_RIGHT_MODULE_Y, Constants.FRONT_RIGHT_AZIMUTH_PID,
                Constants.FRONT_RIGHT_AZIMUTH_ENCODER_OFFSET, RobotMap.FRONT_RIGHT_AZIMUTH_ENCODER_REV,
                Constants.FRONT_RIGHT_VEL_DRIVE_PIDF, tuning, logging, Constants.SwerveModuleName.FRONT_RIGHT);
        this.backLeftModule = new SwerveModule(RobotMap.BACK_LEFT_SWERVE_DRIVE, RobotMap.BACK_LEFT_SWERVE_AZIMUTH,
                RobotMap.BACK_LEFT_SWERVE_AZIMUTH_REV, RobotMap.BACK_LEFT_AZIMUTH_ENCODER, RobotMap.BACK_LEFT_MODULE_X,
                RobotMap.BACK_LEFT_MODULE_Y, Constants.BACK_LEFT_AZIMUTH_PID,
                Constants.BACK_LEFT_AZIMUTH_ENCODER_OFFSET, RobotMap.BACK_LEFT_AZIMUTH_ENCODER_REV,
                Constants.BACK_LEFT_VEL_DRIVE_PIDF, tuning, logging, Constants.SwerveModuleName.BACK_LEFT);
        this.backRightModule = new SwerveModule(RobotMap.BACK_RIGHT_SWERVE_DRIVE, RobotMap.BACK_RIGHT_SWERVE_AZIMUTH,
                RobotMap.BACK_RIGHT_SWERVE_AZIMUTH_REV, RobotMap.BACK_RIGHT_AZIMUTH_ENCODER,
                RobotMap.BACK_RIGHT_MODULE_X, RobotMap.BACK_RIGHT_MODULE_Y, Constants.BACK_RIGHT_AZIMUTH_PID,
                Constants.BACK_RIGHT_AZIMUTH_ENCODER_OFFSET, RobotMap.BACK_RIGHT_AZIMUTH_ENCODER_REV,
                Constants.BACK_RIGHT_VEL_DRIVE_PIDF, tuning, logging, Constants.SwerveModuleName.BACK_RIGHT);

        this.navX = new AHRS(Port.kMXP);
        this.maxModuleRadius = Math.max(Math.max(this.frontLeftModule.getRadius(), this.frontRightModule.getRadius()),
                Math.max(this.backLeftModule.getRadius(), this.backRightModule.getRadius()));
        this.specialMode = SwerveSpecialMode.NORMAL;
    }

    public void zeroYaw() {
        navX.zeroYaw();
    }

    public void drive(double x, double y, double r, boolean fieldOriented) {
        Vector2D strafeVector = Vector2D.vectorFromRectForm(x, y);
        if (fieldOriented) {
            double yaw = navX.getYaw();
            if (!Constants.NAVX_FACING_UP) yaw *= -1;
            strafeVector = strafeVector.rotate(yaw, true);
        }
        Vector2D fLVector = Vector2D.addVectors(strafeVector, getTurnAngleVector(r, frontLeftModule));
        Vector2D fRVector = Vector2D.addVectors(strafeVector, getTurnAngleVector(r, frontRightModule));
        Vector2D bLVector = Vector2D.addVectors(strafeVector, getTurnAngleVector(r, backLeftModule));
        Vector2D bRVector = Vector2D.addVectors(strafeVector, getTurnAngleVector(r, backRightModule));
        double maxLength = Math.max(Math.max(fRVector.getLength(), fLVector.getLength()),
                Math.max(bRVector.getLength(), bLVector.getLength()));
        if (maxLength > 1) {
            fLVector = fLVector.scale(1 / maxLength);
            fRVector = fRVector.scale(1 / maxLength);
            bLVector = bLVector.scale(1 / maxLength);
            bRVector = bRVector.scale(1 / maxLength);
        }
        frontLeftModule.setVector(fLVector);
        frontRightModule.setVector(fRVector);
        backLeftModule.setVector(bLVector);
        backRightModule.setVector(bRVector);
    }

    public void drive(double x, double y, double r) {
        if (this.specialMode == SwerveSpecialMode.RIGHT_TURNING || this.specialMode == SwerveSpecialMode.LEFT_TURNING) {
            double value = Math.hypot(x, y);
            if (this.specialMode == SwerveSpecialMode.RIGHT_TURNING) {
                r = value * 0.42;
            } else {
                r = value * -0.42;
            }
            this.drive(0, value, r, false);
            return;
        }
        this.drive(x, y, r, true);
    }

    public void velocityDriveWithFF(double xVelocity, double yVelocity, double r, double feedForwardVoltage,
            boolean fieldOriented) {
        Vector2D velocityVector = Vector2D.vectorFromRectForm(xVelocity, yVelocity);
        if (fieldOriented) {
            double yaw = navX.getYaw();
            if (!Constants.NAVX_FACING_UP) yaw *= -1;
            velocityVector = velocityVector.rotate(yaw, true);
        }
        //Vector2D scaledVelocityVector = velocityVector.scale(1-Math.abs(rotationPercent));
       // double rotationSpeed = velocityVector.getLength()*rotationPercent;
        // needs to be adjusted for omega
        Vector2D fLVector = Vector2D.addVectors(velocityVector, getTurnAngleVector(velocityVector.getLength()*r, frontLeftModule));
        Vector2D fRVector = Vector2D.addVectors(velocityVector, getTurnAngleVector(velocityVector.getLength()*r, frontRightModule));
        Vector2D bLVector = Vector2D.addVectors(velocityVector, getTurnAngleVector(velocityVector.getLength()*r, backLeftModule));
        Vector2D bRVector = Vector2D.addVectors(velocityVector, getTurnAngleVector(velocityVector.getLength()*r, backRightModule));
        double ff = feedForwardVoltage / Robot.getPDPVoltage();
        frontLeftModule.setVelocityVectorWithFF(fLVector, ff);
        frontRightModule.setVelocityVectorWithFF(fRVector, ff);
        backLeftModule.setVelocityVectorWithFF(bLVector, ff);
        backRightModule.setVelocityVectorWithFF(bRVector, ff);
    }

    public void velocityDriveWithFF(double xVelocity, double yVelocity, double rotationPercent, double feedForwardVoltage) {
        velocityDriveWithFF(xVelocity, yVelocity, rotationPercent, feedForwardVoltage, true);
    }

    public boolean toggleRightTurning() {
        if (this.specialMode != SwerveSpecialMode.RIGHT_TURNING) {
            this.specialMode = SwerveSpecialMode.RIGHT_TURNING;
            return true;
        } else {
            this.specialMode = SwerveSpecialMode.NORMAL;
            return false;
        }
    }

    public boolean toggleLeftTurning() {
        if (this.specialMode != SwerveSpecialMode.LEFT_TURNING) {
            this.specialMode = SwerveSpecialMode.LEFT_TURNING;
            return true;
        } else {
            this.specialMode = SwerveSpecialMode.NORMAL;
            return false;
        }
    }

    public void printEncoderValues() {
        frontLeftModule.printAzimuthEncoderValue();
        frontRightModule.printAzimuthEncoderValue();
        backLeftModule.printAzimuthEncoderValue();
        backRightModule.printAzimuthEncoderValue();
    }

    public void printModuleEncoders(Constants.SwerveModuleName name) {
        SwerveModule module = this.getModule(name);
        module.printAzimuthEncoderValue();
        module.printAzimuthTalonEncoderValue();
    }

    public void setAngle(double degrees) {
        frontLeftModule.setAngleSimple(degrees);
        frontRightModule.setAngleSimple(degrees);
        backLeftModule.setAngleSimple(degrees);
        backRightModule.setAngleSimple(degrees);
    }

    public void zeroEncoders() {
        frontLeftModule.zeroEncoder();
        frontRightModule.zeroEncoder();
        backLeftModule.zeroEncoder();
        backRightModule.zeroEncoder();
    }

    private Vector2D getTurnAngleVector(double r, SwerveModule module) {
        double angle;
        angle = Math.atan2(module.getPositionY() - RobotMap.CENTER_OF_MASS_Y,
                module.getPositionX() - RobotMap.CENTER_OF_MASS_X) - Math.PI / 2;
        // changed from min radius so I need to check
        return new Vector2D(r * module.getRadius() / maxModuleRadius, angle);
    }

    public SwerveModule getModule(Constants.SwerveModuleName name) {
        SwerveModule module;
        switch (name) {
            case FRONT_LEFT:
                module = this.frontLeftModule;
                break;
            case FRONT_RIGHT:
                module = this.frontRightModule;
                break;
            case BACK_LEFT:
                module = this.backLeftModule;
                break;
            case BACK_RIGHT:
                module = this.backRightModule;
                break;
            default:
                module = this.frontLeftModule;
                break;
        }
        return module;
    }

    public void log() {
        frontLeftModule.log();
        frontRightModule.log();
        backLeftModule.log();
        backRightModule.log();
    }

    public void saveLog() {
        frontLeftModule.saveLog();
        frontRightModule.saveLog();
        backLeftModule.saveLog();
        backRightModule.saveLog();
    }

    public void zeroDriveEncoders() {
        frontLeftModule.zeroDriveEncoder();
        frontRightModule.zeroDriveEncoder();
        backLeftModule.zeroDriveEncoder();
        backRightModule.zeroDriveEncoder();
    }

    public double avgEncoderValue() {
        return (Math.abs(frontLeftModule.getDrivePosition()) + Math.abs(frontRightModule.getDrivePosition())
                + Math.abs(backLeftModule.getDrivePosition()) + Math.abs(backRightModule.getDrivePosition())) / 4.0;
    }

    public AHRS getNavX() {
        return this.navX;
    }

    public double getYaw() {
        return Constants.NAVX_FACING_UP ? this.navX.getYaw() : -this.navX.getYaw();
    }

    public void printCurrents() {
        frontLeftModule.printCurrent();
        frontRightModule.printCurrent();
        backLeftModule.printCurrent();
        backRightModule.printCurrent();
    }

    public void startEncoder() {
        this.frontLeftModule.startEncoder();
    }
    public void stopEncoder() {
        this.frontLeftModule.stopEncoder();
    }
}
