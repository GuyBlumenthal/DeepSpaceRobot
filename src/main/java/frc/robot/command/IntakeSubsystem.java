/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.subsystem.IntakeDefaultCommand;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(9);
  WPI_TalonSRX elevatorMotorFollow = new WPI_TalonSRX(10);

  Victor intakeMotor = new Victor(1);

  Victor pivotMotor = new Victor(3);

  DoubleSolenoid lowHatch = new DoubleSolenoid(1, 2);
  DoubleSolenoid highHatch = new DoubleSolenoid(3, 4);

  public IntakeSubsystem() {

    elevatorMotorFollow.setInverted(true);
    elevatorMotorFollow.follow(elevatorMotor);

  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  public void setLowHatch(boolean state) {
    if (state) {
      lowHatch.set(Value.kForward);
    } else {
      lowHatch.set(Value.kReverse);
    }
  }

  public void setHighHatch(boolean state) {
    if (state) {
      highHatch.set(Value.kForward);
    } else {
      highHatch.set(Value.kReverse);
    }
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeDefaultCommand());
  }
}
