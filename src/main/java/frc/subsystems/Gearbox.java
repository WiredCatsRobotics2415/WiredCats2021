package frc.subsystems;

public class Gearbox {
    private Spindexer spindexer;
    private Feeder feeder;

    public Gearbox(Spindexer spindexer, Feeder feeder) {
        this.spindexer = spindexer;
        this.feeder = feeder;
    }

    public void toggleClimber() {
        spindexer.switchClimber();
        feeder.switchClimber();
    }

    public void runClimber(double speed) {
        spindexer.runClimber(speed);
        feeder.runClimber(speed);
    }

}
