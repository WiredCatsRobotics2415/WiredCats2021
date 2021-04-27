package frc.subsystems;

public class Gearbox {
    private Spindexer spindexer;
    private Feeder feeder;

    public Gearbox() {
        this.spindexer = new Spindexer();
        this.feeder = new Feeder();
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
