package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.RobotMap;
import frc.util.pid.TalonFxTunable;

public class Turret {
    private TalonFxTunable leftTalon;
    private TalonFxTunable rightTalon;
    private TalonFxTunable turnTalon;

    private boolean turnReversed;
    private double prevTurnSetpoint;
    private int turns;

    public Turret() {
        this.leftTalon = new TalonFxTunable(new TalonFX(RobotMap.LEFT_TURRET_MOTOR), null, ControlMode.Velocity);
        this.rightTalon = new TalonFxTunable(new TalonFX(RobotMap.RIGHT_TURRET_MOTOR), null, ControlMode.Velocity);
        this.turnTalon = new TalonFxTunable(new TalonFX(RobotMap.TURN_TURRET_MOTOR), null, ControlMode.Position);
        turnReversed = false;
        prevTurnSetpoint = this.turnTalon.getSetpoint();
        turns = 0;
    }

    public void startTurret(double speed) {
        leftTalon.setSetpoint(speed);
        rightTalon.setSetpoint(speed);
        leftTalon.run();
        rightTalon.run();
    }

    public void stopTurret() {
        leftTalon.setSetpoint(0);
        rightTalon.setSetpoint(0);
        leftTalon.run();
        rightTalon.run();
    }

    public void setAngle(double degrees) { 
        if (Math.abs(degrees - this.prevTurnSetpoint) > 90 && Math.abs(degrees - this.prevTurnSetpoint) < 270) {
            degrees = (degrees + 180) % 360;
            turnReversed = true;
        } else {
            turnReversed = false;
        }
        if (degrees > this.prevTurnSetpoint && Math.abs(degrees - this.prevTurnSetpoint) > 180) {
            turns--;
        } else if (degrees < this.prevTurnSetpoint && Math.abs(degrees - this.prevTurnSetpoint) > 180) {
            turns++;
        }
        this.turnTalon.setSetpoint(degrees + turns * 360.0);
        this.turnTalon.run();
        this.prevTurnSetpoint = degrees;
    }
}
