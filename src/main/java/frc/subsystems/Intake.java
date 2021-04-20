package frc.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;

public class Intake {
    //indicate left/right
    DoubleSolenoid leftSolenoid; 
    DoubleSolenoid rightSolenoid; 
    CANSparkMax motor; 

    public Intake() {
        leftSolenoid = new DoubleSolenoid(RobotMap.LEFT_INTAKE_EXTEND, RobotMap.LEFT_INTAKE_RETRACT);
        rightSolenoid = new DoubleSolenoid(RobotMap.RIGHT_INTAKE_EXTEND, RobotMap.RIGHT_INTAKE_RETRACT);
        motor = new CANSparkMax(RobotMap.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
    }
    //2 doublesolenoids for extend/retract
    //1 cansparkmax for running the intake motors
    //TODO: retract, extend, constructor, run, stop methods (all public)
    //look up documentation for these classes to do it
    public void extend() {
        leftSolenoid.set(DoubleSolenoid.Value.kForward);  
        rightSolenoid.set(DoubleSolenoid.Value.kForward); 
    }

    public void startMotor() {
        motor.set(-1.0);
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public void retract() {
        leftSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
