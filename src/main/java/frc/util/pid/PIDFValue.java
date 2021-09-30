package frc.util.pid;

public class PIDFValue extends PIDValue {
    private double kF;

    public PIDFValue(double kP, double kI, double kD, double kF) {
        super(kP, kI, kD);
        this.kF = kF;
    }
    
    public PIDFValue(PIDValue pid) {
        super(pid.getKP(), pid.getKI(), pid.getKD());
        this.kF = 0;
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        super.setPID(kP, kI, kD);
        this.kF = kF;
    }

    public double getKF() {
        return this.kF;
    }

    @Override
    public PIDFValue clone() {
        return new PIDFValue(super.getKP(), super.getKI(), super.getKD(), this.kF);
    }
}