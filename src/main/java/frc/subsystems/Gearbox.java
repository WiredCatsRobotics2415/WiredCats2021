package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gearbox extends SubsystemBase {
    private Spindexer spindexer;
    private Feeder feeder;
    private boolean climber;

    public Gearbox(Spindexer spindexer, Feeder feeder) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.climber = false;
    }

    public void toggleClimber() {
        spindexer.runSpindexer(0);
        feeder.runFeeder(0);
        spindexer.switchClimber();
        feeder.switchClimber();
        climber = !climber;
    }

    public void toggleClimberMove(double speed) {
        if (climber) {
            spindexer.toggleMotor(speed);
            feeder.toggleMotor(-speed);
        }
    }

    public boolean getClimber() {
        return this.climber;
    }

}
