package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.util.pid.PIDFValue;
import frc.util.pid.PIDValue;

public class Constants {
    public static final int kCanTimeoutMs = 30;

    public static final double SWERVE_AZIMUTH_GEAR_RATIO = 56.0 / 3.0;
    public static final double SWERVE_DRIVE_GEAR_RATIO = (28.0/11.0) * 3.0;
    public static final double SWERVE_WHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(3.4);

    public static final double SWERVE_LEFTRIGHT_DISTANCE = Units.inchesToMeters(25.0);
    public static final double SWERVE_FRONTBACK_DISTANCE = Units.inchesToMeters(24.75);

    //kinematics - +x is forward, +y is to the left
    //ordered - frontleft, frontright, backleft, backright
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(SWERVE_FRONTBACK_DISTANCE / 2.0, SWERVE_LEFTRIGHT_DISTANCE / 2.0),
        new Translation2d(SWERVE_FRONTBACK_DISTANCE / 2.0, -SWERVE_LEFTRIGHT_DISTANCE / 2.0),
        new Translation2d(-SWERVE_FRONTBACK_DISTANCE / 2.0, SWERVE_LEFTRIGHT_DISTANCE / 2.0),
        new Translation2d(-SWERVE_FRONTBACK_DISTANCE / 2.0, -SWERVE_LEFTRIGHT_DISTANCE / 2.0));

    public static final PIDValue FRONT_LEFT_AZIMUTH_PID = new PIDValue(0.4, 0.0, 5);
    public static final PIDValue FRONT_RIGHT_AZIMUTH_PID = new PIDValue(0.4, 0.0, 5);
    public static final PIDValue BACK_LEFT_AZIMUTH_PID = new PIDValue(0.4, 0.0, 5);
    public static final PIDValue BACK_RIGHT_AZIMUTH_PID = new PIDValue(0.4, 0.0, 5);
    public static final PIDValue[] AZIMUTH_PID_VALUES = {FRONT_LEFT_AZIMUTH_PID, FRONT_RIGHT_AZIMUTH_PID, BACK_LEFT_AZIMUTH_PID, BACK_RIGHT_AZIMUTH_PID};

    public static final PIDFValue FRONT_LEFT_VEL_DRIVE_PIDF = new PIDFValue(0.10, 0.0, 0.0, 0.0);
    public static final PIDFValue FRONT_RIGHT_VEL_DRIVE_PIDF = new PIDFValue(0.10, 0.0, 0.0, 0.0);
    public static final PIDFValue BACK_LEFT_VEL_DRIVE_PIDF = new PIDFValue(0.10, 0.0, 0.0, 0.0);
    public static final PIDFValue BACK_RIGHT_VEL_DRIVE_PIDF = new PIDFValue(0.10, 0.0, 0.0, 0.0);
    public static final PIDFValue[] DRIVE_PIDF_VALUES = {FRONT_LEFT_VEL_DRIVE_PIDF, FRONT_RIGHT_VEL_DRIVE_PIDF, BACK_LEFT_VEL_DRIVE_PIDF, BACK_RIGHT_VEL_DRIVE_PIDF};

    public static final PIDValue DRIVE_DISTANCE_PID = new PIDValue(8, 0.0, 0.0);
    public static final PIDValue TURNING_PID = new PIDValue(1.0,0.0,0.0);

    public static final PIDValue HOOD_PID = new PIDValue(0.000007, 0.0, 0.0001);
    public static final PIDFValue SHOOTER_PIDF = new PIDFValue(0.2, 0.00001, 0.3, 0.063);
    public static final PIDValue TURRET_PID = new PIDValue(13.0, 0.0, 5.0);

    public static final double KS = 0.776 / 12.0, KV = 2.978 / 12.0, KA = 0.517 / 12.0;// 1.0;

    //look into changing these
    public static final double FRONT_LEFT_AZIMUTH_ENCODER_OFFSET = 142.4;//36.6+90;//36.2+90;//-322.7;//36.2+90;//305.86-180;//304.56 - 180.33;// 304-180; // offset by 90
    public static final double FRONT_RIGHT_AZIMUTH_ENCODER_OFFSET = 51.3;//322-90;//318.9-90;//-50.2;//318.9-90;//72.75+180.0;//163.28 + 180;// 165.24-180;//146.5+180;//5.976+180;
    public static final double BACK_LEFT_AZIMUTH_ENCODER_OFFSET = 104.3;//285.5+90;//288.6+90;//-74.5;//288.6+90;//218;// 219.7;
    public static final double BACK_RIGHT_AZIMUTH_ENCODER_OFFSET = 252.1;//238.4-90;//220-90;//-126.2;//220-90;//212.35;//214.1-4;//211 + 2;// 211.6;
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

    public static final double AUTO_MAX_SPEED = 3;
    public static final double AUTO_MAX_ACCELERATION = 3;
    public static final double AUTO_MAX_ANGULAR_SPEED = Math.PI;
    public static final double AUTO_MAX_ANGULAR_ACCELERATION = Math.PI;
    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(AUTO_MAX_ANGULAR_SPEED, AUTO_MAX_ANGULAR_SPEED);

    //if true then the navX and rio are facing up relative to the field
    public static final boolean NAVX_FACING_UP = false;

    public static final double MAX_SWERVE_SPEED = 5;
    public static final double MAX_ROTATION_SPEED = 9.5;

    public static double degreesToFalcon(double degrees) {
        return degrees / (360.0 / (SWERVE_AZIMUTH_GEAR_RATIO * 2048.0));
    }
    
    public static double falconToDegrees(double ticks) {
        return ticks * (360.0 / (SWERVE_AZIMUTH_GEAR_RATIO * 2048.0));
    }

    public static double falconToRPM(double velocityCounts) {
        return (velocityCounts * (600.0 / 2048.0)) / SWERVE_DRIVE_GEAR_RATIO;
    }

    public static double RPMToFalcon(double RPM) {
        return (RPM * SWERVE_DRIVE_GEAR_RATIO) * (2048.0 / 600.0);
    }

    public static double falconToMPS(double velocitycounts){
        return (falconToRPM(velocitycounts) * SWERVE_WHEEL_CIRCUMFERENCE) / 60.0;
    }

    public static double MPSToFalcon(double velocity){
        return RPMToFalcon((velocity * 60) / SWERVE_WHEEL_CIRCUMFERENCE);
    }
}
