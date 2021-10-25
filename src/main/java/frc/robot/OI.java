package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.Turret;

public class OI {
    private XboxController controller;
    private NetworkTable table;
    private NetworkTableEntry tx, ty, ta;
    private TalonFX tuningMotor;
    private boolean tuning;
    private double distance;
    private double shooterVelocity, offset;
    private double turretAngle, x;
    private double p, d, f;

    public OI(Turret turret) {
        this(0, turret);
    }

    public OI(int port, Turret turret) {
        this.controller = new XboxController(port);
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
        this.tuningMotor = turret.getTurret();
        this.shooterVelocity = turret.getShooterSpeed();
        this.turretAngle = turret.getTurretAngle();
        this.tuning = false;
        this.offset = 0;
        this.x = 0;
        this.p = 0;
        this.d = 0;
        this.f = 0;
    }

    public double getX() {
        double val = this.controller.getRawAxis(0);
        if (Math.abs(val) < Constants.DEADBAND)
            return 0;
        return val;
    }

    public double getY() {
        double val = this.controller.getRawAxis(1) * -1;
        if (Math.abs(val) < Constants.DEADBAND)
            return 0;
        return val;
    }

    public double getRotation() {
        double val = this.controller.getRawAxis(2);
        if (Math.abs(val) < Constants.DEADBAND)
            return 0;
        return val;
    }


    public boolean getAutoAimToggle() {
        return this.controller.getRawButtonPressed(1); // square button
    }

    public boolean getClimberToggle() {
        return this.controller.getRawButtonPressed(4); //triangle button
    }

    public boolean getClimberUp() {
        return this.controller.getRawButtonPressed(6); //right bumper button
    }

    public boolean getClimberDown() {
        return this.controller.getRawButtonPressed(5); //left bumper button
    }
    
    public boolean getIntakeToggle() {
        return this.controller.getRawButtonPressed(2); // x button
    }

    public boolean getShooterToggle() {
        return this.controller.getRawButtonPressed(3); // circle button
    }

    public boolean getRawButtonPressed(int button) {
        return this.controller.getRawButtonPressed(button);
    }

    public double getShooterSpeed() {
        return 0.00967848 * Math.pow((distance + 283.394), 2) + 6240.68 + offset;  
    }

    public double getTurretAngle(Turret turret) {
        return turret.getTurretAngle() + x;
    }

    public void updateShuffleboard(Turret turret) {
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        this.x = tx.getDouble(0.0);
        this.distance = (49.5/Math.tan(27.0+y));

        //post to smart dashboard periodically
        SmartDashboard.putNumber("Limelight/LimelightX", x);
        SmartDashboard.putNumber("Limelight/LimelightY", y);
        SmartDashboard.putNumber("Limelight/LimelightArea", area);

        SmartDashboard.putNumber("Competition/Distance", distance);
        SmartDashboard.putNumber("Competition/Shooter Velocity", this.shooterVelocity);
        SmartDashboard.putNumber("Competition/Turret Angle", this.turretAngle);
        SmartDashboard.putNumber("Competition/Shooter Velocity Offset", this.offset);
        updateCompetition(turret);
        SmartDashboard.putNumber("Competition/Shooter Velocity Actual", turret.getShooterSpeed());
        SmartDashboard.putNumber("Competition/Shooter Velocity Diff", turret.getShooterSpeed()-shooterVelocity + offset);

        SmartDashboard.putNumber("Tuning/P", this.p);
        SmartDashboard.putNumber("Tuning/D", this.d);
        SmartDashboard.putNumber("Tuning/F", this.f);
        if (tuning) updateTuning();
    }


    public void updateCompetition(Turret turret) {
        double newVelocity = SmartDashboard.getNumber("Competition/Shooter Velocity", shooterVelocity);
        if (newVelocity != shooterVelocity) {
            shooterVelocity = newVelocity;
            turret.setShooterSpeed(shooterVelocity + offset);
        }
        double newOffset = SmartDashboard.getNumber("Competition/Shooter Velocity Offset", offset);
        if (newOffset != offset) offset = newOffset;
        double newTurretAngle = SmartDashboard.getNumber("Competition/Turret Angle", turretAngle);
        if (newTurretAngle != turretAngle) {
            turretAngle = newTurretAngle;
            turret.setTurretAngle(turretAngle);
        }
    }
    
    public void updateTuning() {
        double newP = SmartDashboard.getNumber("Tuning/P", p); 
        if (newP != p) {
            p = newP;
            tuningMotor.config_kP(0, p);
        }
        double newD = SmartDashboard.getNumber("Tuning/D", d); 
        if (newD != d) {
            d = newD;
            tuningMotor.config_kD(0, d);
        }
        double newF = SmartDashboard.getNumber("Tuning/F", f); 
        if (newF != f) {
            f = newF;
            tuningMotor.config_kF(0, f);
        }
    }
}