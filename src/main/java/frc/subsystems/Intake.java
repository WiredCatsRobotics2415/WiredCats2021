package frc.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;

public class Intake {
    DoubleSolenoid leftSolenoid; 
    DoubleSolenoid rightSolenoid; 
    boolean running, extended;
    CANSparkMax motor; 

    public Intake() {
        leftSolenoid = new DoubleSolenoid(RobotMap.LEFT_INTAKE_EXTEND, RobotMap.LEFT_INTAKE_RETRACT);
        rightSolenoid = new DoubleSolenoid(RobotMap.RIGHT_INTAKE_EXTEND, RobotMap.RIGHT_INTAKE_RETRACT);
        motor = new CANSparkMax(RobotMap.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
        stopMotor();
        retract();
        running = false;
        extended = false;
    }

    public void extend() {
        leftSolenoid.set(DoubleSolenoid.Value.kForward);  
        rightSolenoid.set(DoubleSolenoid.Value.kForward); 
    }

    public void startMotor() {
        motor.set(0.5);
        running = true;
    }

    public void toggleMotor() {
        if (running) stopMotor();
        else startMotor();
    }
    
    public void toggleExtend() {
        if (extended) retract();
        else extend();
    }

    public void stopMotor() {
        motor.stopMotor();
        running = false;
    }

    public void retract() {
        leftSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
