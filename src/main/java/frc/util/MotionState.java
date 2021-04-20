package frc.util;

public class MotionState {
    public final double time, x, y, theta, velocity, direction, rotationPercent, accel;

    public MotionState(double time, double x, double y, double theta, double velocity, double direction, double rotationPercent,
            double accel) {
        this.time = time;
        this.x = x;
        this.y = y;
        this.theta = Math.toDegrees(Vector2D.modulus(theta, 2*Math.PI));
        this.velocity = velocity;
        this.direction = direction;
        this.rotationPercent = rotationPercent;
        this.accel = accel;
    }

    @Override
    public String toString() {
        return "time:" + time + " x:" + x + " y:" + y + " theta:" + theta + " vel:" + velocity + " direction:"
                + direction + " rotationPercent:" + rotationPercent + " accel:" + accel;
    }
}