package frc.util;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.subsystems.SwerveDrive;
import frc.subsystems.SwerveModule;

public class SwerveOdometry implements Runnable {
    private Vector2D position;
    private double theta;

    private final double thetaOffset;

    private double frLastValue, flLastValue, blLastValue, brLastValue;

    private SwerveModule fr, fl, bl, br;
    private SwerveDrive swerveDrive;

    public SwerveOdometry(SwerveDrive swerveDrive) {
        this(0, 0, 0, swerveDrive);
    }

    public SwerveOdometry(double x, double y, double theta, SwerveDrive swerveDrive) {
        this.position = Vector2D.vectorFromRectForm(x, y);
        this.theta = theta;
        this.fl = swerveDrive.getModule(Constants.SwerveModuleName.FRONT_LEFT);
        this.fr = swerveDrive.getModule(Constants.SwerveModuleName.FRONT_RIGHT);
        this.bl = swerveDrive.getModule(Constants.SwerveModuleName.BACK_LEFT);
        this.br = swerveDrive.getModule(Constants.SwerveModuleName.BACK_RIGHT);
        this.swerveDrive = swerveDrive;
        this.thetaOffset = swerveDrive.getYaw() + this.theta;
        this.frLastValue = fr.getDrivePosition();
        this.flLastValue = fl.getDrivePosition();
        this.blLastValue = bl.getDrivePosition();
        this.brLastValue = br.getDrivePosition();
    }

    private Vector2D getTurningUnitVector(SwerveModule module) {
        Vector2D v = Vector2D.vectorFromRectForm(module.positionX - RobotMap.CENTER_OF_MASS_X,
                module.positionY - RobotMap.CENTER_OF_MASS_Y);
        v = v.scale(1 / v.getLength()); // reduce to 0
        return v.rotate(90, true);
    }

    public void iterate() {
        this.theta = Vector2D.modulus(-swerveDrive.getYaw() + this.thetaOffset,360.0);
        double frValue = fr.getDrivePosition();
        double flValue = fl.getDrivePosition();
        double blValue = bl.getDrivePosition();
        double brValue = br.getDrivePosition();
        Vector2D frChange = new Vector2D(frValue - this.frLastValue, this.fr.getAzimuthAngle(), true);
        Vector2D flChange = new Vector2D(flValue - this.flLastValue, this.fl.getAzimuthAngle(), true);
        Vector2D blChange = new Vector2D(blValue - this.blLastValue, this.bl.getAzimuthAngle(), true);
        Vector2D brChange = new Vector2D(brValue - this.brLastValue, this.br.getAzimuthAngle(), true);
        Vector2D translationVector = Vector2D.addVectors(frChange, flChange, blChange, brChange)
                .scale(0.25 * SwerveModule.UNITS_TO_FT_DRIVE).rotate(this.theta, true);
        double thetaChange = (frChange.dot(getTurningUnitVector(fr)) + flChange.dot(getTurningUnitVector(fl))
                + blChange.dot(getTurningUnitVector(bl)) + brChange.dot(getTurningUnitVector(br))) / 4.0;
        // might use thetaChange later
        this.position = this.position.add(translationVector);
        this.frLastValue = frValue;
        this.flLastValue = flValue;
        this.blLastValue = blValue;
        this.brLastValue = brValue;
    }

    public void run() {
        iterate();
    }

    public double getX() {
        return this.position.getX();
    }

    public double getY() {
        return this.position.getY();
    }

    public Vector2D getPosition() {
        return this.position;
    }

    public double getTheta() {
        return this.theta;
    }

    public double getPercentOutput() {
        return (this.bl.getPercentOutput() + this.br.getPercentOutput() + this.fl.getPercentOutput()
                + this.fr.getPercentOutput()) / 4.0;
    }

    public double getVelocity() {
        double frValue = fr.getDriveVelocity();
        double flValue = fl.getDriveVelocity();
        double blValue = bl.getDriveVelocity();
        double brValue = br.getDriveVelocity();
        Vector2D frVel = new Vector2D(frValue, this.fr.getAzimuthAngle(), true);
        Vector2D flVel = new Vector2D(flValue, this.fl.getAzimuthAngle(), true);
        Vector2D blVel = new Vector2D(blValue, this.bl.getAzimuthAngle(), true);
        Vector2D brVel = new Vector2D(brValue, this.br.getAzimuthAngle(), true);
        return Vector2D.addVectors(frVel, flVel, blVel, brVel).scale(0.25 * SwerveModule.UNITS_TO_FT_DRIVE * 10)
                .getLength();
    }
}