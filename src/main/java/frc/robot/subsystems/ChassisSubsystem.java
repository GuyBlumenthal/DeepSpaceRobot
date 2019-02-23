/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ChassisDefaultCommand;

/**
 * Add your docs here.
 */
public class ChassisSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  UsbCamera camera1;

  WPI_TalonSRX rightMotor;
  WPI_TalonSRX rightMotorFollow;
  
  WPI_TalonSRX leftMotor;
  WPI_TalonSRX leftMotorFollow;

  DifferentialDrive drive;

  public ChassisSubsystem () {

    NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    NetworkTable nt = ntInst.getTable("/SmartDashboard");
    NetworkTableEntry ntEntry= nt.getEntry("Test");
    ntInst.startClientTeam(5834);
    
    ntEntry.setDouble(1.1);

    NetworkTableEntry ntEntry2 = nt.getEntry("Test");
    System.out.print(ntEntry2.getDouble(0.0));

    camera1 = CameraServer.getInstance().startAutomaticCapture();

    rightMotor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_ONE);
    rightMotorFollow = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_TWO);

    rightMotorFollow.follow(rightMotor);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    leftMotor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_ONE);
    leftMotorFollow = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_TWO);

    leftMotorFollow.follow(leftMotor);
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);;
    
    drive = new DifferentialDrive(leftMotor, rightMotor);

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ChassisDefaultCommand());
  }

  public void move(double speed, double turn){
    drive.arcadeDrive(speed, turn);
  }

  public void tankMove (double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void updateSmartDashboard () {

    SmartDashboard.putData("Drive Base", drive);

  }
  
}
