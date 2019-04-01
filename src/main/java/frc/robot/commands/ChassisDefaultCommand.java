/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ChassisDefaultCommand extends Command {
  public ChassisDefaultCommand() {
    requires(Robot.chassisSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
SmartDashboard.putNumber("Command", 0);
    double speed = Robot.oi.getSpeed();
    double turn = Robot.oi.getTurn();

    SmartDashboard.putNumber("Left Speed", 0);
    SmartDashboard.putNumber("Right Speed", 0);
    SmartDashboard.putNumber("currAngle", 0);
    SmartDashboard.putNumber("Difference", 0);

    boolean drivestraight = Robot.oi.getDrive();

    if (drivestraight) {
      Scheduler.getInstance().add(new DriveStraightCommand(14));
    }

    Robot.chassisSubsystem.move(speed, turn);

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
