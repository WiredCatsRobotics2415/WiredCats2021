package frc.util.pid;

import com.revrobotics.CANSparkMax;

import frc.util.PWMAbsoluteEncoder;

public class SparkPositionControllerPWMEncoder implements Runnable {
    public static final double ENCODER_DEADBAND = 2.0;

    private final CANSparkMax motor;
    private final PWMAbsoluteEncoder encoder;
    private final TunablePIDController controller;
    private PIDTuner tuner;
    private double prevEncoderValue;
    private long prevEncoderTime;

    public SparkPositionControllerPWMEncoder(CANSparkMax motor, PWMAbsoluteEncoder encoder, PIDValue pidValue) {
        this.motor = motor;
        this.encoder = encoder;
        this.controller = new TunablePIDController(pidValue);
        this.controller.enableContinuousInput(0, 360);
        this.tuner = null;
        this.prevEncoderValue = 0;
        this.prevEncoderTime = System.currentTimeMillis();
    }

    public SparkPositionControllerPWMEncoder(CANSparkMax motor, PWMAbsoluteEncoder encoder, PIDValue pidValue,
            boolean tuning, String name) {
        this(motor, encoder, pidValue);
        if (tuning) {
            this.tuner = new PIDTuner(this.controller, name);
        }
    }

    public void setSetpoint(double setpoint) {
        if (tuner == null) {
            controller.setSetpoint(setpoint);
        }
    }

    @Override
    public void run() {
        double outputSpeed = 0;
        if (tuner != null) {
            tuner.update();
        }
        if ((Math.abs(prevEncoderValue - encoder.getRotationDegrees()) > ENCODER_DEADBAND
                && Math.abs(prevEncoderValue - encoder.getRotationDegrees()) < (360.0 - ENCODER_DEADBAND))
                || (Math.abs(encoder.getRotationDegrees() - controller.getSetpoint()) > 3.0
                        && Math.abs(encoder.getRotationDegrees() - controller.getSetpoint()) < 357.0)
                || (encoder.getRotationDegrees() - prevEncoderValue)
                        / (System.currentTimeMillis() - prevEncoderTime) < controller.getVelocityError()) {
            prevEncoderValue = encoder.getRotationDegrees(); // bug at 0
            prevEncoderTime = System.currentTimeMillis();
            outputSpeed = controller.calculate(encoder.getRotationDegrees());
            motor.set(outputSpeed);
        }
    }
}
