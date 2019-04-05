/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class TurnToAngle extends Command {
  
  double angle;
  double still;

  public TurnToAngle(double angle) {
    requires(Robot.chassisSubsystem);

    this.angle = angle;
    this.still = 0;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.chassisSubsystem.move(0, 0);
    Robot.chassisSubsystem.resetGyro();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double currAngle = angle + Robot.chassisSubsystem.getAngle();

    double difference = Math.pow(Math.sin(currAngle), 2) * RobotConstants.MAX_DRIVE_STRAIGHT_SPEED;
    double err = Math.abs(0 - currAngle);

    double max = RobotConstants.MAX_DRIVE_STRAIGHT_SPEED - difference;
    double low = -max;

    double leftSpeed = 0, rightSpeed = 0;

    if (err > RobotConstants.ANGLE_DEADBAND) {
      if (currAngle > 0) {
        leftSpeed = low;
        rightSpeed = max;
      } else {
        leftSpeed = max;
        rightSpeed = low;
      }
    }
    SmartDashboard.putNumber("Command", 1);
    Robot.chassisSubsystem.tankMove(leftSpeed, rightSpeed);    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    double currAngle = angle + Robot.chassisSubsystem.getAngle();

    double err = Math.abs(0 - currAngle);

    if (err < RobotConstants.ANGLE_DEADBAND) {
      still ++;
    } else {
      still = 0;
    }

    if (still > 10) {
      return true;
    } else {
      return false;
    }

  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
