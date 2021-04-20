/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.pid;

import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * Add your docs here.
 */
public class TunablePIDController extends PIDController implements PIDTunable {
    private final PIDValue pidValue;

    public TunablePIDController(double kP, double kI, double kD) {
        this(new PIDValue(kP, kI, kD));
    }

    public TunablePIDController(PIDValue pidValue) {
        super(pidValue.getKP(), pidValue.getKI(), pidValue.getKD());
        this.pidValue = pidValue;
    }

    public void setPIDConstants(double kP, double kI, double kD) {
        super.setPID(kP, kI, kD);
        this.pidValue.setPID(kP, kI, kD);
    }

    public double getKP() {
        return this.pidValue.getKP();
    }

    public double getKI() {
        return this.pidValue.getKI();
    }

    public double getKD() {
        return this.pidValue.getKD();
    }

    public double getError() {
        return super.getPositionError();
    }
}