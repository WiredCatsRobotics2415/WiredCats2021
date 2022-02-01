// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.IOException;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.subsystems.Gearbox;
import frc.subsystems.Intake;
import frc.subsystems.SwerveDrive;
import frc.subsystems.Turret;
import frc.subsystems.Spindexer;
import frc.auto.AutoCircle;
import frc.auto.MultiPath;
import frc.subsystems.Feeder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static Intake intake;
    public static Spindexer spindexer;
    public static Feeder feeder;
    private static Gearbox gearbox;
    public static Turret turret;
    private static PowerDistribution pdp;
    private static Compressor compressor;
    public static SwerveDrive swerveDrive;
    private static OI oi;


    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        turret = new Turret();
        swerveDrive = new SwerveDrive();
        intake = new Intake();
        spindexer = new Spindexer();
        feeder = new Feeder();
        gearbox = new Gearbox(spindexer, feeder);
        pdp = new PowerDistribution(RobotMap.PDP_ID, ModuleType.kCTRE);
        //compressor = new Compressor(RobotMap.PCM_ID);
        oi = new OI(turret);
        swerveDrive.zeroYaw();
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
        CommandScheduler.getInstance().schedule(new AutoCircle(swerveDrive));
    }

    public void shootAuto(double delay) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        turret.setTurretAngle(90+tx.getDouble(0.0));
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
        CommandScheduler.getInstance().run();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        intake.retract();
        intake.stopMotor();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        
        //read values periodically
        if (!Constants.ZEROING) {
            swerveDrive.drive(oi.getX(), oi.getY(), oi.getRotation());
            /*if (Math.abs(oi.getTX()) > Constants.DEADBAND && !turning) {
                turret.setTurretAngle(oi.getTurretAngle(turret));
                turning = true;
                counter = 200;
            } else if (counter > 0) counter--;
            else turning = false;
            */
            if (oi.getRawButtonPressed(14)) {
                System.out.println("sync azimuth");
                swerveDrive.syncAzimuth();
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
                swerveDrive.zeroYaw();
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
                swerveDrive.printEncoders();
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
