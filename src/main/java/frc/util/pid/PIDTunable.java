/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.pid;

/**
 * Add your docs here.
 */
public interface PIDTunable {
    void setPIDConstants(double kP, double kI, double kD);

    double getKP();

    double getKI();

    double getKD();

    double getError();

    void setSetpoint(double setpoint);

    double getSetpoint();
}
