/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Add your docs here.
 */
public class OI {

    XboxController drivingController = new XboxController(0);
    XboxController operatingController = new XboxController(1);

    public double getPivotSpeed() {
        return operatingController.getY(Hand.kLeft);
    }

    public double getElevatorSpeed() {
        return operatingController.getY(Hand.kRight);
    }

    public double getInputSpeed() {

        if (operatingController.getBumper(Hand.kRight)) {
            return 1;
        } else if (operatingController.getBumper(Hand.kLeft)) {
            return -1;
        }

        return 0;

    }

    public boolean pushLowHatch() {
        return operatingController.getAButtonPressed();
    }

    public boolean pullLowHatch() {
        return operatingController.getAButtonReleased();
    }

    public boolean pushHighHatch() {
        return operatingController.getBButtonPressed();
    }

    public boolean pullHighHatch() {
        return operatingController.getBButtonReleased();
    }

    public double getScrewSpeed() {

        if (operatingController.getTriggerAxis(Hand.kLeft) > 0.1) {
            return operatingController.getTriggerAxis(Hand.kLeft);
        } else {
            return operatingController.getTriggerAxis(Hand.kRight);
        }

    }
}
