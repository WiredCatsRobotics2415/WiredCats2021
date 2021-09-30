package frc.util.pid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class TalonFxTunable implements PIDTunable {
    private final PIDFValue pidValue;
    private final TalonFX talon;
    private PIDTuner tuner;

    private double setpoint;

    private boolean tuning;
    private boolean testing;
    private ControlMode controlMode;

    public TalonFxTunable(TalonFX talon, double kP, double kI, double kD, ControlMode controlMode) {
        // must pre config talon with PID 
        this(talon, new PIDFValue(kP, kI, kD, 0), controlMode);
    }

    public TalonFxTunable(TalonFX talon, PIDValue pidValue, ControlMode controlMode) {
        this.talon = talon;
        this.pidValue = new PIDFValue(pidValue);
        this.setPID(pidValue.getKP(), pidValue.getKI(), pidValue.getKD());
        this.setpoint = 0;
        this.tuner = null;
        this.tuning = false;
        this.controlMode = controlMode;
        this.testing = false;
    }

    public TalonFxTunable(TalonFX talon, PIDFValue pidValue, ControlMode controlMode) {
        this.talon = talon;
        this.pidValue = pidValue;
        this.setPID(pidValue.getKP(), pidValue.getKI(), pidValue.getKD(), pidValue.getKF());
        this.setpoint = 0;
        this.tuner = null;
        this.tuning = false;
        this.controlMode = controlMode;
        this.testing = false;
    }

    public TalonFxTunable(TalonFX talon, PIDValue pidValue, ControlMode controlMode, boolean tuning, String name) {
        this(talon, pidValue, controlMode);
        this.tuning = tuning;
        if (this.tuning) {
            this.tuner = new PIDTuner(this, name);
        }
    }

    public TalonFxTunable(boolean testing, TalonFX talon, PIDValue pidValue, ControlMode controlMode) {
        this.talon = talon;
        this.pidValue = new PIDFValue(pidValue);
        this.controlMode = controlMode;
        this.tuner = null;
        this.tuning = false;
        this.setpoint = 0;
        this.testing = testing;
        this.setPID(pidValue.getKP(), pidValue.getKI(), pidValue.getKD());
    }

    public void setPID(double kP, double kI, double kD) {
        if(!testing) {
            this.talon.config_kP(0, pidValue.getKP(), Constants.kCanTimeoutMs);
            this.talon.config_kI(0, pidValue.getKI(), Constants.kCanTimeoutMs);
            this.talon.config_kD(0, pidValue.getKD(), Constants.kCanTimeoutMs);
        }
    }

    public void setPID(double kP, double kI, double kD, double kF) {
        if(!testing) {
            this.talon.config_kP(0, pidValue.getKP(), Constants.kCanTimeoutMs);
            this.talon.config_kI(0, pidValue.getKI(), Constants.kCanTimeoutMs);
            this.talon.config_kD(0, pidValue.getKD(), Constants.kCanTimeoutMs);
            this.talon.config_kF(0, pidValue.getKF(), Constants.kCanTimeoutMs);
        }
    }

    public void setPIDConstants(double kP, double kI, double kD) {
        this.setPID(kP, kI, kD);
        this.pidValue.setPID(kP, kI, kD);
    }

    public double getKP() {
        return this.pidValue.getKP();
    }

    public double getKI() {
        return this.pidValue.getKI();
    }

    public double getKD() {
        return this.pidValue.getKD();
    }

    public double getKF() {
        return this.pidValue.getKF();
    }

    public double getError() {
        if(testing) return 0;
        return talon.getClosedLoopError(0);
    }

    @Override
    public double getSetpoint() {
        return this.setpoint;
    }

    public void enableTuning(String name) {
        if (tuning)
            return;
        this.tuning = true;
        this.tuner = new PIDTuner(this, name);
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        if(!testing) {
            talon.set(this.controlMode, setpoint);
        }
    }

    public void setSetpointWithFF(double setpoint, double ff) {
        this.setpoint = setpoint;
        if(!testing) {
            talon.set(this.controlMode, setpoint, DemandType.ArbitraryFeedForward, ff);
        }
    }
    public void run() {
        if(tuning && !testing) {
            this.tuner.update();
        }
    }
}