/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeDefaultCommand extends Command {
  public IntakeDefaultCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intakeSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double elevatorSpeed = Robot.oi.getElevatorSpeed();
    double pivotSpeed = Robot.oi.getPivotSpeed();

    double intakeSpeed = Robot.oi.getInputSpeed();

    boolean pullLowHatch = Robot.oi.pullLowHatch();
    boolean pushLowHatch = Robot.oi.pushLowHatch();

    boolean pullHighHatch = Robot.oi.pullHighHatch();
    boolean pushHighHatch = Robot.oi.pushHighHatch();

    Robot.intakeSubsystem.setElevatorSpeed(elevatorSpeed);
    Robot.intakeSubsystem.setPivotSpeed(pivotSpeed);
    Robot.intakeSubsystem.setIntake(intakeSpeed);

    if (pushLowHatch) {
      Robot.intakeSubsystem.setLowHatch(true);
    } else if (pullLowHatch) {
      Robot.intakeSubsystem.setLowHatch(false);
    }

    if (pushHighHatch) {
      Robot.intakeSubsystem.setHighHatch(true);
    } else if (pullHighHatch) {
      Robot.intakeSubsystem.setHighHatch(false);
    }

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
