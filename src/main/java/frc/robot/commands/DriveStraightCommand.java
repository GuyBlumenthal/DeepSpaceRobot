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

public class DriveStraightCommand extends Command {

  double time;

  public DriveStraightCommand(double time) {
    requires(Robot.chassisSubsystem);

    this.time = time;

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

    double difference = Math.pow(Math.sin(currAngle), 2) * RobotConstants.DRIVE_STRAIGHT_MAX_SPEED;
    double err = Math.abs(0 - currAngle);

    double max = RobotConstants.DRIVE_STRAIGHT_MAX_SPEED;
    double low = max - difference;

    double leftSpeed = max, rightSpeed = max;

    if (err > RobotConstants.ANGLE_DEADBAND) {
      if (currAngle > 0) {
        leftSpeed = low;
      } else {
        rightSpeed = low;
      }
    }
    SmartDashboard.putNumber("Command", 1);
    Robot.chassisSubsystem.tankMove(leftSpeed, rightSpeed);

    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);
    SmartDashboard.putNumber("currAngle", currAngle);
    SmartDashboard.putNumber("Difference", difference);
    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timeSinceInitialized() >= time;
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
