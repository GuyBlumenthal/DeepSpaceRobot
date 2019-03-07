/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.LiftDefaultCommand;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  Victor screwMotor;
  Victor liftDrivingMotor;

  DigitalInput screwLimitSwitch;

  public LiftSubsystem () {

    screwMotor  = new Victor(RobotMap.SCREW_MOTOR);
    liftDrivingMotor = new Victor(RobotMap.LIFT_DRIVING_MOTOR);

    screwLimitSwitch = new DigitalInput(RobotMap.SCREW_LIMIT_SWITCH);

  }

  public boolean getScrewLimitSwitch() {
    return !screwLimitSwitch.get();
  }
  public void setScrewSpeed(double speed){
    
    if (getScrewLimitSwitch() && speed > 0) {
      speed = 0;
    }

    screwMotor.set(speed);
  }
  
  public void setLiftDrivingSpeed(double liftDrivingSpeed) {
    liftDrivingMotor.set(liftDrivingSpeed);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new LiftDefaultCommand());
  }

  public void updateSmartDashboard () {

  }
  
}
