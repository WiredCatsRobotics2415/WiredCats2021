package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    private XboxController controller;

    public OI() {
        this(0);
    }

    public OI(int port) {
        this.controller = new XboxController(port);
    }

    public double getX() {
        double val = this.controller.getRawAxis(0);
        if (Math.abs(val) < Constants.DEADBAND)
            return 0;
        return val;
    }

    public double getY() {
        double val = this.controller.getRawAxis(1) * -1;
        if (Math.abs(val) < Constants.DEADBAND)
            return 0;
        return val;
    }

    public double getRotation() {
        double val = this.controller.getRawAxis(2);
        if (Math.abs(val) < Constants.DEADBAND)
            return 0;
        return val;
    }


    public boolean getAutoAimToggle() {
        return this.controller.getRawButtonPressed(1); // square button
    }

    public boolean getClimberToggle() {
        return this.controller.getRawButtonPressed(4); //triangle button
    }

    public boolean getClimberUp() {
        return this.controller.getRawButtonPressed(6); //right bumper button
    }

    public boolean getClimberDown() {
        return this.controller.getRawButtonPressed(5); //left bumper button
    }
    
    public boolean getIntakeToggle() {
        return this.controller.getRawButtonPressed(2); // x button
    }

    public boolean getIntakeExtensionToggle() {
        return this.controller.getRawButtonPressed(3); // circle button
    }

    public boolean getTurretToggle() {
        return this.controller.getRawButtonPressed(0); //right joystick click
    }

    /*
    public boolean getCompressorToggle() {
        return this.controller.getRawButtonPressed(4); // triangle button
    }
    */

    public boolean getRightTurningToggle() {
        return this.controller.getRawButtonPressed(6); // back right
    }

    public boolean getLeftTurningToggle() {
        return this.controller.getRawButtonPressed(5); // back right
    }

    public boolean getRawButtonPressed(int button) {
        return this.controller.getRawButtonPressed(button);
    }

    public void updateShuffleboard() {
        
    }
    
    public double getTurretSpeed() {
        return 0.00967848 * Math.pow((distance + 283.394), 2) + 6240.68;  
    }
}