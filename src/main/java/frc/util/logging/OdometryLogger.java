package frc.util.logging;

import frc.util.SwerveOdometry;

public class OdometryLogger implements Loggable {
    private SwerveOdometry odom;

    public OdometryLogger(SwerveOdometry odom) {
        this.odom = odom;
    }

    @Override
    public double[] getLogOutput() {
        double[] arr = new double[5];
        arr[0] = odom.getX();
        arr[1] = odom.getY();
        arr[2] = odom.getTheta();
        arr[3] = odom.getVelocity();
        arr[4] = odom.getPercentOutput();
        return arr;
    }
}