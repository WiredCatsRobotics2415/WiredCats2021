// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.Gearbox;
import frc.subsystems.Intake;
import frc.subsystems.SwerveDrive;
import frc.subsystems.Turret;
import frc.util.CSVReader;
import frc.util.PathFollowerController;
import frc.util.SwerveOdometry;
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
    public static PathFollowerController pathController;
    public static SwerveOdometry odometry;
    public static final String saveName = "WiredCats2021";
    private int pathCount = 0;
    int counter = 10000;
    boolean turning = false;


    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        turret = new Turret();
        swerveDrive = new SwerveDrive(Constants.SWERVE_TUNING, Constants.SWERVE_LOGGING);
        intake = new Intake();
        spindexer = new Spindexer();
        feeder = new Feeder();
        gearbox = new Gearbox(spindexer, feeder);
        pdp = new PowerDistributionPanel(RobotMap.PDP_ID);
        compressor = new Compressor(RobotMap.PCM_ID);
        oi = new OI(turret);
        swerveDrive.zeroYaw();
        counter = 10;
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
        turret.setTurretAngle(90);
        shootAuto(3);
        //CSVReader csvReader = new CSVReader(Filesystem.getDeployDirectory() + "/grits3balldone.csv");
        //pathController = new PathFollowerController(swerveDrive, csvReader.getValues(), Constants.KS, Constants.KV,
          //  Constants.KA, 1, Constants.DRIVE_DISTANCE_PID, Constants.TURNING_PID, true);
        //intake.extend();
        //intake.startMotor(0.5);
        //spindexer.toggleMotor(0.5);
        //pathController.start();
    }

    public void shootAuto(double delay) {
        Timer.delay(5);
        turret.setTurretAngle(oi.getTurretAngle(turret));
        turret.toggleShooterSpeed(oi.getShooterSpeed());
        Timer.delay(2.5);
        feeder.runFeeder(0.5);
        spindexer.runSpindexer(0.5);
        Timer.delay(delay);
        turret.toggleShooterSpeed(0);
        feeder.runFeeder(0);
        spindexer.runSpindexer(0);

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        if (counter > 0) {
            swerveDrive.drive(0, -1, 0);
            counter--;
        } else {
            swerveDrive.drive(0, 0, 0);
        }
        //if (!pathController.getFinished()) pathController.run();
        /*else if (pathCount < 1) {
            CSVReader csvReader = new CSVReader(Filesystem.getDeployDirectory() + "/grits6balltrenchdone.csv");
            pathController = new PathFollowerController(swerveDrive, csvReader.getValues(), Constants.KS, Constants.KV,
                Constants.KA, 1, Constants.DRIVE_DISTANCE_PID, Constants.TURNING_PID, true);
            pathController.start();
            pathCount++;
        } else {
            intake.stopMotor();
            intake.retract();
            shootAuto(3);
            turret.setTurretAngle(0);
        }
        */
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        swerveDrive.stopEncoder();
        turret.startEncoder();
        turret.zeroTurret();
        turret.stopEncoder();
        swerveDrive.startEncoder();
        intake.retract();
        intake.stopMotor();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        
        //read values periodically
        if (!Constants.ZEROING) {
            swerveDrive.drive(oi.getX(), oi.getY(), oi.getRotation());
            if (Math.abs(oi.getTX()) > Constants.DEADBAND && !turning) {
                turret.setTurretAngle(oi.getTurretAngle(turret));
                turning = true;
                counter = 200;
            } else if (counter > 0) counter--;
            else turning = false;
            if (oi.getRawButtonPressed(14)) {
                System.out.println("zero Encoders");
                swerveDrive.zeroEncoders();
            }
            if (oi.getIntakeToggle() && !gearbox.getClimber()) {
                intake.toggleExtend();
                intake.toggleMotor(0.5);
            }
            if (oi.getClimberToggle()) {
                gearbox.toggleClimber();
            }
            //if (oi.getAutoAimToggle()) {
              //  turret.setTurretAngle(oi.getTurretAngle(turret));
            //}
            if (oi.getShooterToggle()) {
                turret.toggleShooterSpeed(oi.getShooterSpeed());
                feeder.runFeeder(0.5);
                if (!turret.getShooterRunning()) feeder.runFeeder(0);
            }
            if (oi.getRawButtonPressed(10) && !gearbox.getClimber()) {
                feeder.toggleMotor(0.5);
            }
            if (oi.getRawButtonPressed(9) ) {
                intake.toggleMotor(-0.5);
            }
            if (oi.getClimberDown()) {
                if (gearbox.getClimber()) gearbox.toggleClimberMove(-0.5);
                else if (turret.getShooterRunning()) {
                    feeder.runFeeder(0.5); 
                    spindexer.toggleMotor(-0.5);
                }
                else spindexer.toggleMotor(-0.5);
            } else if (oi.getClimberUp()) {
                if (gearbox.getClimber()) gearbox.toggleClimberMove(0.5);
                else if (turret.getShooterRunning()) {
                    feeder.runFeeder(0.5); 
                    spindexer.toggleMotor(0.5);
                }
                else spindexer.toggleMotor(0.5);
            }
            spindexer.checkCurrent();
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
        }
        oi.updateShuffleboard(turret);
        SmartDashboard.putNumber("Spindexer Current", spindexer.getCurrent());
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
