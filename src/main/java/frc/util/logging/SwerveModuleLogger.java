package frc.util.logging;

import frc.subsystems.SwerveModule;

public class SwerveModuleLogger implements Loggable {
    public enum SwerveModuleLoggerMode {
        DRIVE, DRIVE_WCURRENT, AZIMUTH, BOTH;
    };

    private SwerveModule module;
    private SwerveModuleLoggerMode mode;

    public SwerveModuleLogger(SwerveModule module) {
        this.module = module;
        this.mode = SwerveModuleLoggerMode.BOTH;
    }

    public SwerveModuleLogger(SwerveModule module, SwerveModuleLoggerMode mode) {
        this.module = module;
        this.mode = mode;
    }

    @Override
    public double[] getLogOutput() {
        switch (this.mode) {
            case DRIVE:
                return this.getDriveOutput();
            case DRIVE_WCURRENT:
                return this.getDriveWCurrentOutput();
            case AZIMUTH:
                return this.getAzimuthOutput();
            case BOTH:
                return this.getBothOutput();
            default:
                return this.getBothOutput();
        }
    }

    private double[] getDriveOutput() {
        double[] arr = new double[2];
        arr[0] = module.getDrivePosition();
        arr[1] = module.getDriveVoltage();
        return arr;
    }

    private double[] getDriveWCurrentOutput() {
        double[] arr = new double[3];
        arr[0] = module.getDrivePosition();
        arr[1] = module.getDriveVoltage();
        arr[2] = module.getDriveCurrent();
        return arr;
    }

    private double[] getAzimuthOutput() {
        double[] arr = new double[2];
        arr[0] = module.getAzimuthAngle();
        arr[1] = module.getAzimuthVoltage();
        return arr;
    }

    private double[] getBothOutput() {
        double[] arr = new double[4];
        arr[0] = module.getDrivePosition();
        arr[1] = module.getDriveVoltage();
        arr[2] = module.getAzimuthAngle();
        arr[3] = module.getAzimuthVoltage();
        return arr;
    }
}