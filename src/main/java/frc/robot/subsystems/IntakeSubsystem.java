/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeDefaultCommand;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Victor intakeMotor;

  Victor pivotMotor;

  DoubleSolenoid lowHatch;
  DoubleSolenoid highHatch;

  public IntakeSubsystem () {

    intakeMotor = new Victor(RobotMap.INTAKE_MOTOR);
  
    pivotMotor = new Victor(RobotMap.PIVOT_MOTOR);
  
    lowHatch = new DoubleSolenoid(RobotMap.LOW_HATCH_SOLENOID_PORT_ONE, RobotMap.LOW_HATCH_SOLENOID_PORT_TWO);
    highHatch = new DoubleSolenoid(RobotMap.HIGH_HATCH_SOLENOID_PORT_ONE, RobotMap.HIGH_HATCH_SOLENOID_PORT_TWO);

  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  public void setLowHatch(boolean state) {
    if (state) {
      lowHatch.set(Value.kReverse);
    } else {
      lowHatch.set(Value.kForward);
    }
  }

  public void setHighHatch(boolean state) {
    if (state) {
      highHatch.set(Value.kReverse);
    } else {
      highHatch.set(Value.kForward);
    }
  }

  public boolean getHighHatch() {
    return highHatch.get() == Value.kForward ? true : false;
  }

  public boolean getLowHatch() {
    return lowHatch.get() == Value.kForward ? true : false;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeDefaultCommand());
  }

  public void updateSmartDashboard () {

    SmartDashboard.putBoolean("High Hatch Solenoids", getHighHatch());
    SmartDashboard.putBoolean("Low Hatch Solenoids", getLowHatch());

  }

}
