// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.Gearbox;
import frc.subsystems.Intake;
import frc.subsystems.SwerveDrive;
import frc.subsystems.Turret;
import frc.subsystems.Spindexer;
import frc.subsystems.Feeder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static Intake intake;
    private static Spindexer spindexer;
    private static Feeder feeder;
    private static Gearbox gearbox;
    private static Turret turret;
    private static PowerDistributionPanel pdp;
    private static Compressor compressor;
    private static SwerveDrive swerveDrive;
    private static OI oi;
    public static final String saveName = "WiredCats2021";
    private double shooterVelocity;
    private double hoodAngle;
    private double turretAngle;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        //swerveDrive = new SwerveDrive(Constants.SWERVE_TUNING, Constants.SWERVE_LOGGING);
        //intake = new Intake();
        spindexer = new Spindexer();
        feeder = new Feeder();
        gearbox = new Gearbox(spindexer, feeder);
        turret = new Turret();
        pdp = new PowerDistributionPanel(RobotMap.PDP_ID);
        //compressor = new Compressor(RobotMap.PCM_ID);
        oi = new OI();
        this.shooterVelocity = 0;
        SmartDashboard.putNumber("Shooter Velocity", this.shooterVelocity);
        this.hoodAngle = turret.getHoodAngle();
        SmartDashboard.putNumber("Hood Angle", this.hoodAngle);
        this.turretAngle = turret.getTurretAngle();
        SmartDashboard.putNumber("Turret Angle", this.turretAngle);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if (!Constants.ZEROING) {
            /*
            if (oi.getRightTurningToggle()) {
                System.out.println("Right turning: " + swerveDrive.toggleRightTurning());
            } else if (oi.getLeftTurningToggle()) {
                System.out.println("Left turning: " + swerveDrive.toggleLeftTurning());
            }
            */
            //swerveDrive.drive(oi.getX(), oi.getY(), oi.getRotation());
            if (oi.getRawButtonPressed(14)) {
                System.out.println("zero Encoders");
                //swerveDrive.zeroEncoders();
                turret.zeroTurret();
                turret.zeroHood();
            }
            if (oi.getIntakeToggle() && !gearbox.getClimber()) {
                intake.toggleExtend();
                intake.toggleMotor();
            }
            if (oi.getClimberToggle()) {
                gearbox.toggleClimber();
            }
            if (oi.getFeederToggle()) {
                feeder.toggleMotor(1.0);
            }
            if (oi.getClimberDown()) {
                if (gearbox.getClimber()) gearbox.toggleClimberMove(-0.5);
                else spindexer.toggleMotor(-0.25);
            } else if (oi.getClimberUp()) {
                if (gearbox.getClimber()) gearbox.toggleClimberMove(0.5);
                else spindexer.toggleMotor(0.25);
            }
        } else {
            if (oi.getRawButtonPressed(1)) {
                swerveDrive.printModuleEncoders(Constants.SwerveModuleName.FRONT_LEFT);
            }
            if (oi.getRawButtonPressed(2)) {
                swerveDrive.printModuleEncoders(Constants.SwerveModuleName.FRONT_RIGHT);
            }
            if (oi.getRawButtonPressed(3)) {
                swerveDrive.printModuleEncoders(Constants.SwerveModuleName.BACK_LEFT);
            }
            if (oi.getRawButtonPressed(4)) {
                swerveDrive.printModuleEncoders(Constants.SwerveModuleName.BACK_RIGHT);
            }
            if (oi.getRawButtonPressed(9)) {
                turret.printEncoderValues();
            }
        }
        if (Math.abs(turret.getHoodSetpoint()-turret.getHoodAngle()) < Constants.DEADBAND) turret.stopHood(); 
        double newVelocity = SmartDashboard.getNumber("Shooter Velocity", shooterVelocity);
        if (newVelocity != shooterVelocity) {
            shooterVelocity = newVelocity;
            turret.setShooterSpeed(shooterVelocity);
        }
        SmartDashboard.putNumber("Shooter Velocity Actual", turret.getShooterSpeed());
        double newHoodAngle = SmartDashboard.getNumber("Hood Angle", hoodAngle);
        if (newHoodAngle != hoodAngle) {
            hoodAngle = newHoodAngle;
            turret.setHoodAngle(hoodAngle);
        }
        SmartDashboard.putNumber("Hood Angle Actual", turret.getHoodAngle());
        double newTurretAngle = SmartDashboard.getNumber("Turret Angle", turretAngle);
        if (newTurretAngle != turretAngle) {
            turretAngle = newTurretAngle;
            turret.setTurretAngle(turretAngle);
        }
        SmartDashboard.putNumber("Turret Angle Actual", turret.getTurretAngle());
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    public static double getPDPVoltage() {
        return pdp.getVoltage();
    }
}
