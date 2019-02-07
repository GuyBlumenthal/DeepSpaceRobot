/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.subsystem.ChassisDefaultCommand;

/**
 * Add your docs here.
 */
public class ChassisSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  WPI_TalonSRX rightMotor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_ONE);
  WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_TWO);
  
  WPI_TalonSRX leftMotor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_ONE);
  WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_TWO);

  SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMotor, rightMotor2);
  SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMotor, leftMotor2);

  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ChassisDefaultCommand());
  }

  public void move(double speed, double turn){
    drive.arcadeDrive(speed, turn);
  }
}
