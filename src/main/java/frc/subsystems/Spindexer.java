package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Spindexer {
    private TalonFX motor;
    private Solenoid solenoid; 
    private boolean climber;
    boolean running;

    public Spindexer() {
        this.motor = new TalonFX(RobotMap.RIGHT_GEARBOX_MOTOR);
        
        this.motor.configFactoryDefault(Constants.kCanTimeoutMs);
        this.motor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Constants.kCanTimeoutMs);
        this.motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCanTimeoutMs);
        this.motor.configSelectedFeedbackCoefficient(1.0/3.0, 0, Constants.kCanTimeoutMs);
        this.motor.setNeutralMode(NeutralMode.Brake);
        this.motor.setInverted(false);
        this.motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .1));
        this.motor.configNeutralDeadband(Constants.MOTORMIN, Constants.kCanTimeoutMs);

        this.solenoid = new Solenoid(RobotMap.RIGHT_GEARBOX_PISTON);
        this.solenoid.set(false);

        this.climber = false;
        running = false;
    }

    public void switchClimber() {
        this.climber = !climber;
        this.solenoid.set(climber);
        this.motor.set(ControlMode.PercentOutput, 0);
        running = false;
    }

    public void runSpindexer(double speed) {
        if (!climber) {
            this.motor.set(ControlMode.PercentOutput, speed);
            if (speed > 0) running = true;
        }
    }

    public void runClimber(double speed) {
        if (!climber) {
            this.motor.set(ControlMode.PercentOutput, speed);
            if (speed > 0) running = true;
        }
    }

    public void toggleMotor(double speed) {
        if (running) {
            this.motor.set(ControlMode.PercentOutput, 0);
            running = false;
        } else {
            this.motor.set(ControlMode.PercentOutput, speed);
            running = true;
        }
    }
    
    public boolean getRunning() {
        return this.running;
    }

    public boolean getClimber() {
        return this.climber;
    }

    public double getCurrent() {
        return this.motor.getSupplyCurrent();
    }
}