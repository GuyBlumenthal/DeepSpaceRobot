/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorDefaultCommand;

/**
 * Add your docs here.
 */
public class ElevatorSubsystem extends Subsystem {

  WPI_TalonSRX elevatorMotor;
  WPI_TalonSRX elevatorMotorFollow;

  DigitalInput lowLimitSwitch;
  DigitalInput highLimitSwitch;

  public ElevatorSubsystem() {

    lowLimitSwitch = new DigitalInput(RobotMap.LOW_LIMIT_SWITCH);
    highLimitSwitch = new DigitalInput(RobotMap.HIGH_LIMIT_SWITCH);

    elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_LEAD);
    elevatorMotorFollow = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_FOLLOW);
  }

  public boolean getLowLimitSwitch() {
    return !lowLimitSwitch.get();
  }
  
  public boolean getHighLimitSwitch() {
    return !highLimitSwitch.get();
  }

  public void setElevatorSpeed(double speed) {
  
    if (speed < 0 && getLowLimitSwitch()) {
      speed = 0;
    }

    if (speed > 0 && getHighLimitSwitch()) {
      speed = 0;
    }

    elevatorMotor.set(speed);
    elevatorMotorFollow.set(speed * 0.95);

  }

  public void moveElevatorUp() {
    setElevatorSpeed(RobotConstants.ELEVATOR_COMMAND_SPEED);
  }

  public void moveElevatorDown() {
    setElevatorSpeed(-RobotConstants.ELEVATOR_COMMAND_SPEED);
  }

  public void stopElevatorMovement() {
    setElevatorSpeed(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevatorDefaultCommand());
  }

  public void updateSmartdashboard () {

    SmartDashboard.putBoolean("High limit switch", getHighLimitSwitch());
    SmartDashboard.putBoolean("Low limit switch", getLowLimitSwitch());

  }

}
