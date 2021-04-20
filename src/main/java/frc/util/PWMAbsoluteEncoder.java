package frc.util;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter.Mode;

public class PWMAbsoluteEncoder {
    private static final int SAMPLE_TIME = 1026; // 1026 microseconds for 10 bit PWM
    private static final int COUNTS_PER_ROTATION = 1024;

    private double offset;
    private final DigitalInput encoderInput;
    private final Counter encoderRawInputUp;
    private final Counter encoderRawInputDown;
    private boolean reversed;
    private boolean testing;

    public PWMAbsoluteEncoder(int channel) {
        this.encoderInput = new DigitalInput(channel);

        this.encoderRawInputUp = new Counter(Mode.kSemiperiod); // right mode for this application
        this.encoderRawInputUp.setUpSource(this.encoderInput);// select the port
        this.encoderRawInputUp.setSemiPeriodMode(true); // measure from rising edge to falling edge

        this.encoderRawInputDown = new Counter(Mode.kSemiperiod); // right mode for this application
        this.encoderRawInputDown.setUpSource(this.encoderInput);// select the port
        this.encoderRawInputDown.setSemiPeriodMode(false); // measure from rising edge to falling edge

        this.offset = 0;
        this.reversed = false;
        this.testing = false;
    }

    public PWMAbsoluteEncoder(int channel, double degreesOffset, boolean reversed) {
        this(channel);
        setOffset(degreesOffset);
        this.reversed = reversed;
    }

    public PWMAbsoluteEncoder(boolean testing, int channel) {
        if(!testing) {
            this.encoderInput = new DigitalInput(channel);

            this.encoderRawInputUp = new Counter(Mode.kSemiperiod); // right mode for this application
            this.encoderRawInputUp.setUpSource(this.encoderInput);// select the port
            this.encoderRawInputUp.setSemiPeriodMode(true); // measure from rising edge to falling edge

            this.encoderRawInputDown = new Counter(Mode.kSemiperiod); // right mode for this application
            this.encoderRawInputDown.setUpSource(this.encoderInput);// select the port
            this.encoderRawInputDown.setSemiPeriodMode(false); // measure from rising edge to falling edge

            this.offset = 0;
            this.reversed = false;
        } else {
            this.encoderInput = null;
            this.encoderRawInputDown = null;
            this.encoderRawInputUp = null;
            this.offset = 0;
            this.reversed = false;
            this.testing = true;
        }
    }

    public void setOffset(double degreeOffset) {
        this.offset = (degreeOffset / 360 * COUNTS_PER_ROTATION) % COUNTS_PER_ROTATION;
        if (this.offset < 0)
            this.offset += COUNTS_PER_ROTATION;
    }

    public double getRotationRaw() { // out of 1024 for 10 bit PWM
        double timeOn, timeOff;
        if(!testing) {
            timeOn = encoderRawInputUp.getPeriod() * 1000000; // get time in seconds and convert to microseconds
            timeOff = encoderRawInputDown.getPeriod() * 1000000;
        } else {
            timeOn = 0;
            timeOff = 1024;
        }
        double x = ((timeOn * 1026) / (timeOn + timeOff)) - 1;
        double returnValue = 0;
        if (x <= 1023)
            returnValue = x;
        else if (x > 1023)
            returnValue = 1023;
        else {
            returnValue = 0;
            System.out.println("Error reading encoder value, value = " + x + "   " + offset);
        }
        if (this.reversed) {
            returnValue *= -1;
        }
        returnValue = (returnValue - offset) % COUNTS_PER_ROTATION;
        if (returnValue < 0)
            returnValue += COUNTS_PER_ROTATION;
        return returnValue;
    }

    public double getRotationPercent() {
        return getRotationRaw() / COUNTS_PER_ROTATION;
    }

    public double getRotationDegrees() {
        return getRotationPercent() * 360;
    }
}
