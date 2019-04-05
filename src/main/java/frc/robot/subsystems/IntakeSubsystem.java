/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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

  DoubleSolenoid hatch;

  DigitalInput pivotLimitSwitch;

  public IntakeSubsystem() {

    intakeMotor = new Victor(RobotMap.INTAKE_MOTOR);

    pivotMotor = new Victor(RobotMap.PIVOT_MOTOR);

    hatch = new DoubleSolenoid(RobotMap.HATCH_SOLENOID_PORT_ONE, RobotMap.HATCH_SOLENOID_PORT_TWO);

    pivotLimitSwitch = new DigitalInput(RobotMap.PIVOT_LIMIT_SWITCH);

  }

  public boolean getPivotLimitSwitch() {
    return !pivotLimitSwitch.get();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setPivotSpeed(double speed) {
    
    if (getPivotLimitSwitch() && speed > 0) {
      speed = 0;
    }

    pivotMotor.set(speed);
  }

  public void lockHatch(boolean state) {
    if (state) {
      hatch.set(Value.kReverse);
    } else {
      hatch.set(Value.kForward);
    }
  }
  
  public boolean getHatch() {

    return hatch.get() == Value.kForward ? false : true;

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeDefaultCommand());
  }

  public void updateSmartDashboard() {

    SmartDashboard.putBoolean("Hatch Solenoid", getHatch());

    SmartDashboard.putBoolean("Pivot Limit Switch", getPivotLimitSwitch());

  }

}
