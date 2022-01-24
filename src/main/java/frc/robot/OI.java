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
    private double p, d, f, i;
    private double setValue;

    public OI(Turret turret) {
        this(0, turret);
    }

    public OI(int port, Turret turret) {
        this.controller = new XboxController(port);
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
        this.tuningMotor = turret.getShooter();
        this.shooterVelocity = turret.getShooterSpeed();
        this.turretAngle = turret.getTurretAngle();
        this.tuning = true;
        this.offset = 0;
        this.setValue = 0;
        this.x = 0;
        this.p = 0;
        this.d = 0;
        this.i = 0;
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
        if (distance >= 120) return 0.026781 * Math.pow((distance -16.3842), 2) + 6997.53 + offset;
        else return Math.min(-19.1176 * distance + 8500.49, 7200);
    }

    public double getTurretAngle(Turret turret) {
        this.x = tx.getDouble(0.0);
        return turret.getTurretAngle() + this.x;
    }
    public double getTX() {
        return tx.getDouble(0.0);
    }

    public void updateShuffleboard(Turret turret) {
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        this.x = tx.getDouble(0.0);
        this.distance = (49.5/Math.tan((25.0+y)*(Math.PI/180.0)));

        //post to smart dashboard periodically
        SmartDashboard.putNumber("Limelight/LimelightX", x);
        SmartDashboard.putNumber("Limelight/LimelightY", y);
        SmartDashboard.putNumber("Limelight/LimelightArea", area);

        SmartDashboard.putNumber("Competition/Distance", distance);
        updateCompetition(turret);
        SmartDashboard.putNumber("Competition/Shooter Velocity", this.shooterVelocity);
        SmartDashboard.putNumber("Competition/Turret Angle", this.turretAngle);
        SmartDashboard.putNumber("Competition/Turret Angle Actual", turret.getTurretAngle());
        SmartDashboard.putNumber("Competition/Shooter Velocity Offset", this.offset);
        SmartDashboard.putNumber("Competition/Shooter Velocity Actual", turret.getShooterSpeed());
        SmartDashboard.putNumber("Competition/Shooter Velocity Diff", turret.getShooterSpeed()-turret.getShooterSetpoint() + offset);

        SmartDashboard.putNumber("Tuning/Value", turret.getShooterSpeed());
        if (tuning) updateTuning(turret);
        SmartDashboard.putNumber("Tuning/SetValue", this.setValue);
        SmartDashboard.putNumber("Tuning/P", this.p);
        SmartDashboard.putNumber("Tuning/D", this.d);        
        SmartDashboard.putNumber("Tuning/I", this.i);
        SmartDashboard.putNumber("Tuning/F", this.f);
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
            //turret.setTurretAngle(turretAngle);
        }
    }
    
    public void updateTuning(Turret turret) {
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
        double newI = SmartDashboard.getNumber("Tuning/I", i); 
        if (newI != i) {
            i = newI;
            tuningMotor.config_kI(0, i);
        }
        double newF = SmartDashboard.getNumber("Tuning/F", f); 
        if (newF != f) {
            f = newF;
            tuningMotor.config_kF(0, f);
        }
        double newValue = SmartDashboard.getNumber("Tuning/SetValue", setValue);
        if (newValue != setValue) {
            setValue = newValue;
            turret.setShooterSpeed(setValue);
        }
    }
}