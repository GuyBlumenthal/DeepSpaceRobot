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

    public double getSpeed() {
        return -drivingController.getY(Hand.kLeft);
    }

    public double getTurn() {
        return drivingController.getX(Hand.kRight);
    }

    public double getPivotSpeed() {
        return -operatingController.getY(Hand.kRight);
    }

    public double getElevatorSpeed() {
        return -operatingController.getY(Hand.kLeft);
    }

    public boolean getDrive () {
        return drivingController.getBButtonPressed();
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
        return operatingController.getAButton();
    }

    public boolean pushHighHatch() {
        return operatingController.getBButton();
    }

    public double getScrewSpeed() {

        return operatingController.getTriggerAxis(Hand.kRight) - 
        operatingController.getTriggerAxis(Hand.kLeft);

    }

    public double getLiftDrivingSpeed() {

        return drivingController.getTriggerAxis(Hand.kRight) - 
        drivingController.getTriggerAxis(Hand.kLeft);

    }
}