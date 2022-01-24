package frc.robot;

public class RobotMap {

    public static final int FRONT_LEFT_SWERVE_AZIMUTH = 32;
    public static final int FRONT_RIGHT_SWERVE_AZIMUTH = 20;
    public static final int BACK_LEFT_SWERVE_AZIMUTH = 21;
    public static final int BACK_RIGHT_SWERVE_AZIMUTH = 23;
    public static final int[] AZIMUTH_PORTS = {FRONT_LEFT_SWERVE_AZIMUTH, FRONT_RIGHT_SWERVE_AZIMUTH, BACK_LEFT_SWERVE_AZIMUTH, BACK_RIGHT_SWERVE_AZIMUTH};

    public static final boolean FRONT_LEFT_SWERVE_AZIMUTH_REV = false;
    public static final boolean FRONT_RIGHT_SWERVE_AZIMUTH_REV = false;
    public static final boolean BACK_LEFT_SWERVE_AZIMUTH_REV = false;
    public static final boolean BACK_RIGHT_SWERVE_AZIMUTH_REV = false;
    public static final boolean[] AZIMUTH_REVERSED = {FRONT_LEFT_SWERVE_AZIMUTH_REV, FRONT_RIGHT_SWERVE_AZIMUTH_REV, BACK_LEFT_SWERVE_AZIMUTH_REV, BACK_RIGHT_SWERVE_AZIMUTH_REV};

    public static final boolean FRONT_LEFT_SWERVE_DRIVE_REV = false;//true
    public static final boolean FRONT_RIGHT_SWERVE_DRIVE_REV = false;
    public static final boolean BACK_LEFT_SWERVE_DRIVE_REV = false;//true
    public static final boolean BACK_RIGHT_SWERVE_DRIVE_REV = false;
    public static final boolean[] DRIVE_REVERSED = {FRONT_LEFT_SWERVE_DRIVE_REV, FRONT_RIGHT_SWERVE_DRIVE_REV, BACK_LEFT_SWERVE_DRIVE_REV, BACK_RIGHT_SWERVE_DRIVE_REV};

    public static final int FRONT_LEFT_SWERVE_DRIVE = 22;
    public static final int FRONT_RIGHT_SWERVE_DRIVE = 35;
    public static final int BACK_LEFT_SWERVE_DRIVE = 34;
    public static final int BACK_RIGHT_SWERVE_DRIVE = 33;
    public static final int[] DRIVE_PORTS = {FRONT_LEFT_SWERVE_DRIVE, FRONT_RIGHT_SWERVE_DRIVE, BACK_LEFT_SWERVE_DRIVE, BACK_RIGHT_SWERVE_DRIVE};

    public static final int FRONT_LEFT_AZIMUTH_ENCODER = 2;
    public static final int FRONT_RIGHT_AZIMUTH_ENCODER = 3;
    public static final int BACK_LEFT_AZIMUTH_ENCODER = 0;
    public static final int BACK_RIGHT_AZIMUTH_ENCODER = 1;
    public static final int[] AZIMUTH_ENCODER_PORTS = {FRONT_LEFT_AZIMUTH_ENCODER, FRONT_RIGHT_AZIMUTH_ENCODER, BACK_LEFT_AZIMUTH_ENCODER, BACK_RIGHT_AZIMUTH_ENCODER};

    public static final boolean FRONT_LEFT_AZIMUTH_ENCODER_REV = true;
    public static final boolean FRONT_RIGHT_AZIMUTH_ENCODER_REV = true;
    public static final boolean BACK_LEFT_AZIMUTH_ENCODER_REV = true;
    public static final boolean BACK_RIGHT_AZIMUTH_ENCODER_REV = true;
    public static final boolean[] AZIMUTH_ENCODER_REV = {FRONT_LEFT_AZIMUTH_ENCODER_REV, FRONT_RIGHT_AZIMUTH_ENCODER_REV, BACK_LEFT_AZIMUTH_ENCODER_REV, BACK_RIGHT_AZIMUTH_ENCODER_REV};

    // Intake
    public static final int INTAKE_MOTOR = 18;
    public static final int LEFT_INTAKE = 5;
    public static final int RIGHT_INTAKE = 7;

    public static final int LEFT_TURRET_MOTOR = 38;
    public static final int RIGHT_TURRET_MOTOR = 39;
    public static final int TURN_TURRET_MOTOR = 37;
    public static final int HOOD_TURRET_MOTOR = 19;
    public static final int TURN_TURRET_ENCODER = 5;
    public static final int HOOD_TURRET_ENCODER = 4;
    public static final boolean TURN_TURRET_ENCODER_REV = false;
    public static final boolean HOOD_TURRET_ENCODER_REV = false;

    public static final int LEFT_GEARBOX_MOTOR = 36;
    public static final int RIGHT_GEARBOX_MOTOR = 40;
    public static final int LEFT_GEARBOX_PISTON = 4;
    public static final int RIGHT_GEARBOX_PISTON = 6;

    // PCM
    public static final int PCM_ID = 6;
    public static final int PDP_ID = 5;
}