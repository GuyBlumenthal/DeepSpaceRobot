/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class DriveStraightCommand extends Command {

  double cmDistance;

  public DriveStraightCommand(double cmDistance) {
    requires(Robot.chassisSubsystem);

    this.cmDistance = cmDistance;

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

    double currAngle = Robot.chassisSubsystem.getAngle();

    double difference = Math.cos(currAngle) * RobotConstants.DRIVE_STRAIGHT_MAX_SPEED;
    double err = Math.abs(0 - currAngle);

    double max = RobotConstants.DRIVE_STRAIGHT_MAX_SPEED;
    double low = max - difference;

    double leftSpeed = max, rightSpeed = max;

    if (err > RobotConstants.ANGLE_DEADBAND) {
      if (currAngle > 0) {
        // Right is max
        leftSpeed = low;
      } else {
        // Left is max
        rightSpeed = low;
      }
    }

    Robot.chassisSubsystem.tankMove(leftSpeed, rightSpeed);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
