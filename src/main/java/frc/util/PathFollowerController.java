package frc.util;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystems.SwerveDrive;
import frc.util.logging.MotorLogger;
import frc.util.logging.OdometryLogger;
import frc.util.pid.PIDValue;
import frc.util.pid.TunablePIDController;

public class PathFollowerController implements Runnable {
    private final double kS, kV, kA;
    private final TunablePIDController xController;
    private final TunablePIDController yController;
    private final TunablePIDController thetaController;

    private MotorLogger logger;
    private double lookAheadTime;
    private MotionState[] trajectory;
    private SwerveDrive swerveDrive;
    private SwerveOdometry odometry;
    private long startTime;
    private int index;
    private boolean logged;
    private boolean done;

    // trajectory should be in feet and degrees
    public PathFollowerController(SwerveDrive swerveDrive, MotionState[] trajectory, double kS, double kV, double kA,
            double lookAheadTime, PIDValue distancePID, PIDValue turningPID, boolean logging) {
        this.trajectory = trajectory.clone();
        this.swerveDrive = swerveDrive;
        if (this.trajectory.length == 0) {
            System.err.println("empty trajectory");
        } else {
            this.odometry = new SwerveOdometry(this.trajectory[0].x, this.trajectory[0].y, this.trajectory[0].theta,
                    swerveDrive);
        }
        this.xController = new TunablePIDController(distancePID);
        this.xController.setSetpoint(0);
        this.yController = new TunablePIDController(distancePID);
        this.yController.setSetpoint(0);
        this.thetaController = new TunablePIDController(turningPID);
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.logged = true;
        this.lookAheadTime = lookAheadTime;
        this.startTime = System.currentTimeMillis();
        this.index = 0;
        if (logging) {
            this.logger = new MotorLogger(new OdometryLogger(this.odometry));
            this.logged = false;
        } else
            this.logger = null;
        this.done = false;
    }

    public PathFollowerController(SwerveDrive swerveDrive, double[][] trajectory, double kS, double kV, double kA,
            double lookAheadTime, PIDValue distancePID, PIDValue turningPID) {
        this(swerveDrive, convertFromArray(trajectory), kS, kV, kA, lookAheadTime, distancePID, turningPID, false);
    }

    public PathFollowerController(SwerveDrive swerveDrive, double[][] trajectory, double kS, double kV, double kA,
            double lookAheadTime, PIDValue distancePID, PIDValue turningPID, boolean logging) {
        this(swerveDrive, convertFromArray(trajectory), kS, kV, kA, lookAheadTime, distancePID, turningPID, logging);
    }

    public void start() {
        this.startTime = System.currentTimeMillis();
        run();
    }

    public void run() {
        odometry.iterate();
        if (this.logger != null)
            this.logger.run();
        long time = System.currentTimeMillis();
        this.index = getClosestStateIndex(time);
        if ((time - startTime) / 1000.0 > this.trajectory[this.trajectory.length - 1].time) {
            if (!this.logged) {
                this.saveLog();
                this.logged = true;
            }
            this.swerveDrive.drive(0, 0, 0);
            done = true;
            return;
        }
        MotionState currentState = this.trajectory[this.index];
        double direction = currentState.direction; // in degrees
        Vector2D velocity = new Vector2D(currentState.velocity, direction, true);
        double xError = odometry.getX() - currentState.x;
        double yError = odometry.getY() - currentState.y;
        double thetaError = odometry.getTheta() - currentState.theta;
        xError = this.xController.calculate(xError);
        yError = this.yController.calculate(yError);
        thetaError = this.thetaController.calculate(thetaError);
        direction = velocity.getAngleDeg();
        double feedForwardVoltage = this.kS * Math.signum(velocity.getLength()) + this.kV * velocity.getLength()
                + this.kA * currentState.accel;
        Vector2D feedForwardVector = new Vector2D(feedForwardVoltage, direction, true);
        Vector2D xErrorVector = new Vector2D(xError, 0, true);
        Vector2D yErrorVector = new Vector2D(yError, 90, true);
        //System.out.println("1" + feedForwardVector.getLength());
        feedForwardVector = Vector2D.addVectors(feedForwardVector, xErrorVector, yErrorVector);
        //System.out.println("2" + feedForwardVector.getLength());
        //double rotatePercent = currentState.rotationPercent;
        double theta = swerveDrive.getYaw();
        /*if(Math.signum(thetaError) == Math.signum(rotatePercent)) {
            feedForwardVector = feedForwardVector.scale(1+Math.abs(thetaError));
            rotatePercent += thetaError;
        } else {
            feedForwardVector = feedForwardVector.scale(1-Math.abs(thetaError));
            rotatePercent += thetaError;
        }*/
        velocity = new Vector2D(velocity.getLength(), feedForwardVector.getAngleDeg(), true);
        //System.out.println("3" + feedForwardVector.getLength());
        //velocity = feedForwardVector.scale(1-rotatePercent);
        //System.out.println("4" + feedForwardVector.getLength());
        //this.swerveDrive.velocityDriveWithFF(velocity.getX(), velocity.getY(), theta*-0.05,//-currentState.rotationPercent,
        //        feedForwardVector.getLength()); //rotation is opposite for swerve drive
        feedForwardVector = feedForwardVector.scale(1.0 / Robot.getPDPVoltage());
        this.swerveDrive.drive(feedForwardVector.getX(), feedForwardVector.getY(), theta*-0.005);
    }

    private int getClosestStateIndex(long time) { // in millis
        int closestIndex = this.index;
        double timeError = Double.MAX_VALUE;
        time -= this.startTime;
        double seconds = time / 1000.0;
        if (closestIndex == this.trajectory.length - 1)
            return -1;
        for (int i = closestIndex; i < this.trajectory.length; i++) { // find the closest time in the trajectory
            if (Math.abs(this.trajectory[i].time - seconds) > timeError) {
                closestIndex = i;
                break;
            }
            timeError = Math.abs(this.trajectory[i].time - seconds);
        }
        return closestIndex;
    }

    public void saveLog() {
        if (this.logger != null) {
            //System.out.println("odom logged");
            this.logger.saveDataToCSV(Robot.saveName);
            //this.logger.saveDataToCSV("odom" + Constants.KA + "A" + Constants.KS + "S" + Constants.KV + "V"
            //        + Constants.DRIVE_DISTANCE_PID.getKP() + "P" + ".csv");
        }
    }

    private static MotionState[] convertFromArray(double[][] array) {
        return convertFromArray(array, 0, 1, 2, 3, 4, 5, 6, 7, 8);
    }

    private static MotionState[] convertFromArray(double[][] array, int timeIndex, int xIndex, int yIndex,
            int thetaIndex, int velocityIndex, int directionIndex, int omegaIndex, int accelIndex, int alphaIndex) {
        MotionState[] trajectory = new MotionState[array.length];
        for (int i = 0; i < trajectory.length; i++) {
            try {
                trajectory[i] = new MotionState(array[i][timeIndex], array[i][xIndex], array[i][yIndex],
                        array[i][thetaIndex], array[i][velocityIndex], array[i][directionIndex], array[i][omegaIndex],
                        array[i][accelIndex]);
            } catch (ArrayIndexOutOfBoundsException e) {
                System.err.println("array trajectory does not meet specified array config");
            }
        }
        return trajectory;
    }

    public boolean getFinished() {
        return this.done;
    }
}