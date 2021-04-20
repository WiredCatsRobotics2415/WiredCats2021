package frc.util.pid;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PIDTuner {
    private final PIDTunable controller;
    private final ShuffleboardTab tuner;
    private final NetworkTableEntry kPEntry;
    private final NetworkTableEntry kIEntry;
    private final NetworkTableEntry kDEntry;
    private final NetworkTableEntry errorEntry;
    private final NetworkTableEntry setpointEntry;

    public PIDTuner(PIDTunable controller, String name) {
        this.controller = controller;
        this.tuner = Shuffleboard.getTab(name);
        this.kPEntry = this.tuner.add("kP", this.controller.getKP()).getEntry();
        this.kIEntry = this.tuner.add("kI", this.controller.getKI()).getEntry();
        this.kDEntry = this.tuner.add("kD", this.controller.getKD()).getEntry();
        this.errorEntry = this.tuner.add("Error", 0).getEntry();
        this.setpointEntry = this.tuner.add("Setpoint", 0).getEntry();
    }

    public void update() {
        double kP = kPEntry.getDouble(controller.getKP());
        double kI = kIEntry.getDouble(controller.getKI());
        double kD = kDEntry.getDouble(controller.getKD());
        double setpoint = setpointEntry.getDouble(controller.getSetpoint());
        controller.setPIDConstants(kP, kI, kD);
        controller.setSetpoint(setpoint);
        errorEntry.forceSetNumber(controller.getError());
    }
}