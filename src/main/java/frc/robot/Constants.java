package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.util.pid.PIDFValue;
import frc.util.pid.PIDValue;

public class Constants {
    public static final int kCanTimeoutMs = 30;

    public static final PIDValue FRONT_LEFT_AZIMUTH_PID = new PIDValue(31, 0.0, 70);
    public static final PIDValue FRONT_RIGHT_AZIMUTH_PID = new PIDValue(28, 0.0, 65);
    public static final PIDValue BACK_LEFT_AZIMUTH_PID = new PIDValue(22, 0.0, 66);
    public static final PIDValue BACK_RIGHT_AZIMUTH_PID = new PIDValue(30, 0.0, 55);

    public static final PIDFValue FRONT_LEFT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue FRONT_RIGHT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue BACK_LEFT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue BACK_RIGHT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);

    public static final PIDValue DRIVE_DISTANCE_PID = new PIDValue(8, 0.0, 0.0);
    public static final PIDValue TURNING_PID = new PIDValue(1.0,0.0,0.0);

    public static final double KS = 0.164, KV = 0.97, KA = 0.1;// 1.0;

    public static final double FRONT_LEFT_AZIMUTH_ENCODER_OFFSET = 303.4-180;//305.86-180;//304.56 - 180.33;// 304-180; // offset by 90
    public static final double FRONT_RIGHT_AZIMUTH_ENCODER_OFFSET = 48.8+180;//72.75+180.0;//163.28 + 180;// 165.24-180;//146.5+180;//5.976+180;
    public static final double MOTORMIN = 0.05;
    public static final double BACK_LEFT_AZIMUTH_ENCODER_OFFSET = 219.02;//218;// 219.7;
    public static final double BACK_RIGHT_AZIMUTH_ENCODER_OFFSET = 285.4;//212.35;//214.1-4;//211 + 2;// 211.6;
    public static final double DEADBAND = 0.1;

    public static NeutralMode DRIVE_BREAK_MODE = NeutralMode.Brake;

    public static boolean ZEROING = false;

    public enum SwerveModuleName {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT;
    }

    // tuning
    public static final boolean SWERVE_TUNING = false;

    // logging
    public static final boolean SWERVE_LOGGING = true;
    public static final SwerveModuleLoggerMode SWERVE_LOGGING_MODE = SwerveModuleLoggerMode.DRIVE_WCURRENT;

}
