package frc.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    Solenoid leftSolenoid;
    Solenoid rightSolenoid;
    boolean running, extended;
    CANSparkMax motor;
    ParallelCommandGroup intakeCommand;

    public Intake() {
        leftSolenoid = new Solenoid(RobotMap.PCM_ID, PneumaticsModuleType.CTREPCM, RobotMap.LEFT_INTAKE);
        rightSolenoid = new Solenoid(RobotMap.PCM_ID, PneumaticsModuleType.CTREPCM, RobotMap.RIGHT_INTAKE);
        motor = new CANSparkMax(RobotMap.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
        stopMotor();
        retract();
        running = false;
        extended = false;
        this.intakeCommand = new ParallelCommandGroup(new InstantCommand(() -> toggleExtend()),
                new InstantCommand(() -> toggleMotor(0.5)));
    }

    public void extend() {
        leftSolenoid.set(true);
        rightSolenoid.set(true);
        this.extended = true;
    }

    public void startMotor(double speed) {
        motor.set(speed);
        running = true;
    }

    public void toggleMotor(double speed) {
        if (running)
            stopMotor();
        else
            startMotor(speed);
    }

    public void toggleExtend() {
        if (extended)
            retract();
        else
            extend();
    }

    public void stopMotor() {
        motor.stopMotor();
        running = false;
    }

    public void retract() {
        leftSolenoid.set(false);
        rightSolenoid.set(false);
        this.extended = false;
    }

    public ParallelCommandGroup getIntakeCommand() {
        return this.intakeCommand;
    }
}
