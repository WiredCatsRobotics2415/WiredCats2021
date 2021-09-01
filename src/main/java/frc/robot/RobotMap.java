package frc.robot;

public class RobotMap {

    public static final int FRONT_LEFT_SWERVE_AZIMUTH = 32;
    public static final int FRONT_RIGHT_SWERVE_AZIMUTH = 20;
    public static final int BACK_LEFT_SWERVE_AZIMUTH = 21;
    public static final int BACK_RIGHT_SWERVE_AZIMUTH = 23;

    public static final boolean FRONT_LEFT_SWERVE_AZIMUTH_REV = true;
    public static final boolean FRONT_RIGHT_SWERVE_AZIMUTH_REV = true;
    public static final boolean BACK_LEFT_SWERVE_AZIMUTH_REV = true;
    public static final boolean BACK_RIGHT_SWERVE_AZIMUTH_REV = true;

    public static final int FRONT_LEFT_SWERVE_DRIVE = 22;
    public static final int FRONT_RIGHT_SWERVE_DRIVE = 35;
    public static final int BACK_LEFT_SWERVE_DRIVE = 34;
    public static final int BACK_RIGHT_SWERVE_DRIVE = 33;

    public static final int FRONT_LEFT_AZIMUTH_ENCODER = 2;
    public static final int FRONT_RIGHT_AZIMUTH_ENCODER = 3;
    public static final int BACK_LEFT_AZIMUTH_ENCODER = 0;
    public static final int BACK_RIGHT_AZIMUTH_ENCODER = 1;

    public static final boolean FRONT_LEFT_AZIMUTH_ENCODER_REV = true;
    public static final boolean FRONT_RIGHT_AZIMUTH_ENCODER_REV = true;
    public static final boolean BACK_LEFT_AZIMUTH_ENCODER_REV = true;
    public static final boolean BACK_RIGHT_AZIMUTH_ENCODER_REV = true;

    public static final double FRONT_LEFT_MODULE_X = -18.79/2.0, FRONT_LEFT_MODULE_Y = 18.79/2.0; // inches
    public static final double FRONT_RIGHT_MODULE_X = 18.79/2.0, FRONT_RIGHT_MODULE_Y = 18.79/2.0; // quick values for sqare robot
    public static final double BACK_LEFT_MODULE_X = -18.79/2.0, BACK_LEFT_MODULE_Y = -18.79/2.0;
    public static final double BACK_RIGHT_MODULE_X = 18.79/2.0, BACK_RIGHT_MODULE_Y = -18.79/2.0;

    public static final double CENTER_OF_MASS_X = 0;
    public static final double CENTER_OF_MASS_Y = 0;

    // Intake
    public static final int INTAKE_MOTOR = 4;
    public static final int LEFT_INTAKE_EXTEND = 0;
    public static final int LEFT_INTAKE_RETRACT = 0;
    public static final int RIGHT_INTAKE_EXTEND = 0;
    public static final int RIGHT_INTAKE_RETRACT = 0;

    public static final int LEFT_TURRET_MOTOR = 38;
    public static final int RIGHT_TURRET_MOTOR = 39;
    public static final int TURN_TURRET_MOTOR = 37;
    public static final int HOOD_TURRET_MOTOR = 9;
    public static final int TURN_TURRET_ENCODER = 0;
    public static final int HOOD_TURRET_ENCODER = 0;
    public static final boolean TURN_TURRET_ENCODER_REV = false;
    public static final boolean HOOD_TURRET_ENCODER_REV = false;

    public static final int LEFT_GEARBOX_MOTOR = 36;
    public static final int RIGHT_GEARBOX_MOTOR = 40;
    public static final int LEFT_GEARBOX_PISTON = 0;
    public static final int RIGHT_GEARBOX_PISTON = 0;

    // PCM
    public static final int PCM_ID = 10;
    public static final int PDP_ID = 5;
}