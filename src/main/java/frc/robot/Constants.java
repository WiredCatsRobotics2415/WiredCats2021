package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.util.logging.SwerveModuleLogger.SwerveModuleLoggerMode;
import frc.util.pid.PIDFValue;
import frc.util.pid.PIDValue;

public class Constants {
    public static final int kCanTimeoutMs = 30;

    /**
     * @deprecated
     */
    public static final double SWERVE_AZIMUTH_TICKS_TO_DEGREE = 360.0 / (2048 * 56.0 / 3.0);
    public static final double SWERVE_AZIMUTH_GEAR_RATIO = 2048.0 / 40320.0;

    public static final PIDValue FRONT_LEFT_AZIMUTH_PID = new PIDValue(9, 0.0, 0.4);
    public static final PIDValue FRONT_RIGHT_AZIMUTH_PID = new PIDValue(9, 0.0, 0.4);
    public static final PIDValue BACK_LEFT_AZIMUTH_PID = new PIDValue(9, 0.0, 0.4);
    public static final PIDValue BACK_RIGHT_AZIMUTH_PID = new PIDValue(9, 0.0, 0.4);
    public static final PIDValue[] AZIMUTH_PID_VALUES = {FRONT_LEFT_AZIMUTH_PID, FRONT_RIGHT_AZIMUTH_PID, BACK_LEFT_AZIMUTH_PID, BACK_RIGHT_AZIMUTH_PID};

    public static final PIDFValue FRONT_LEFT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue FRONT_RIGHT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue BACK_LEFT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue BACK_RIGHT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue[] DRIVE_PIDF_VALUES = {FRONT_LEFT_VEL_DRIVE_PIDF, FRONT_RIGHT_VEL_DRIVE_PIDF, BACK_LEFT_VEL_DRIVE_PIDF, BACK_RIGHT_VEL_DRIVE_PIDF};

    public static final PIDValue DRIVE_DISTANCE_PID = new PIDValue(8, 0.0, 0.0);
    public static final PIDValue TURNING_PID = new PIDValue(1.0,0.0,0.0);

    public static final PIDValue HOOD_PID = new PIDValue(0.000007, 0.0, 0.0001);
    public static final PIDFValue SHOOTER_PIDF = new PIDFValue(0.2, 0.00001, 0.3, 0.063);
    public static final PIDValue TURRET_PID = new PIDValue(13.0, 0.0, 5.0);

    public static final double KS = 0.164, KV = 0.97, KA = 0.1;// 1.0;

    public static final double FRONT_LEFT_AZIMUTH_ENCODER_OFFSET = 145.5+90;//36.6+90;//36.2+90;//-322.7;//36.2+90;//305.86-180;//304.56 - 180.33;// 304-180; // offset by 90
    public static final double FRONT_RIGHT_AZIMUTH_ENCODER_OFFSET = 8.8-90;//322-90;//318.9-90;//-50.2;//318.9-90;//72.75+180.0;//163.28 + 180;// 165.24-180;//146.5+180;//5.976+180;
    public static final double BACK_LEFT_AZIMUTH_ENCODER_OFFSET = 286.2+90;//285.5+90;//288.6+90;//-74.5;//288.6+90;//218;// 219.7;
    public static final double BACK_RIGHT_AZIMUTH_ENCODER_OFFSET = 235.9-90;//238.4-90;//220-90;//-126.2;//220-90;//212.35;//214.1-4;//211 + 2;// 211.6;
    public static final double[] ENCODER_OFFSETS = {FRONT_LEFT_AZIMUTH_ENCODER_OFFSET, FRONT_RIGHT_AZIMUTH_ENCODER_OFFSET, BACK_LEFT_AZIMUTH_ENCODER_OFFSET, BACK_RIGHT_AZIMUTH_ENCODER_OFFSET};
    
    public static final double MOTORMIN = 0.05;
    public static final double DEADBAND = 0.2;

    public static final double TURN_TURRET_ENCODER_OFFSET = 0;
    public static final double HOOD_TURRET_ENCODER_OFFSET = 0;

    public static NeutralMode DRIVE_BREAK_MODE = NeutralMode.Brake;

    public static boolean ZEROING = false;

    public enum SwerveModuleName {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT;
    }

    // tuning
    public static final boolean SWERVE_TUNING = false;

    // logging
    public static final boolean SWERVE_LOGGING = false;
    public static final SwerveModuleLoggerMode SWERVE_LOGGING_MODE = SwerveModuleLoggerMode.DRIVE_WCURRENT;

    //if true then the navX and rio are facing up relative to the field
    public static final boolean NAVX_FACING_UP = false;

    public static double degreesToFalcon(double degrees) {
        return (degrees / 360.0) * 2048.0;
    }

    public static double MPSToFalcon(double mps) {
        return 
    }
}
