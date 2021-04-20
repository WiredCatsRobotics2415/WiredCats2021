package frc.util.logging;

import com.kauailabs.navx.frc.AHRS;

public class NavXLogger implements Loggable {
    public enum NavXLoggerMode {
        ACCELERATION
    };

    private AHRS navX;
    private NavXLoggerMode mode;

    public NavXLogger(AHRS navX) {
        this.navX = navX;
        this.mode = NavXLoggerMode.ACCELERATION;
    }

    public NavXLogger(AHRS navX, NavXLoggerMode mode) {
        this.navX = navX;
        this.mode = mode;
    }

    @Override
    public double[] getLogOutput() {
        switch (this.mode) {
            case ACCELERATION:
                return this.getDriveOutput();
            default:
                return this.getDriveOutput();
        }
    }

    private double[] getDriveOutput() {
        double[] arr = new double[3];
        arr[0] = this.navX.getWorldLinearAccelX();
        arr[1] = this.navX.getWorldLinearAccelY();
        arr[2] = this.navX.getWorldLinearAccelZ();
        return arr;
    }
}