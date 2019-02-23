/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import frc.robot.*;

public class FollowCommand extends Command {
 	
	static double[] parameters = new double[3]; //0 = distance, 1 = angle, 2 = orientation
	double speedL, speedR, maxSpeedL, maxSpeedR, minSpeedL, minSpeedR;
	double l = 11; // change
	
	public class CompareParams implements Comparator<double[][]> {

		@Override
		public int compare(double[][] param1A, double[][] param2A) {
			
			double[] param1 = param1A[0];
			double[] param2 = param2A[0];
			
			double a = ((param1[0]/parameters[0]) + (param1[1]/parameters[1]) + (param1[2]/parameters[2])) / 3;
			double b = ((param2[0]/parameters[0]) + (param2[1]/parameters[1]) + (param2[2]/parameters[2])) / 3;
			return Double.compare(a, b);
		}
		
	}
	
	/*
	 * Configure these two functions on current surface.
	 */
	
	public double toEncoderCounts(double inches) {
		return 0;
	}
	
	public double toInches(double encoderCounts) {
		return 0;
	}
	
	private double[] checkPosition(double parameters[], double speedL, double speedR) {
		double[] deltas = new double[3];
		
		double alpha = ((90 - parameters[1]) + parameters[2] + 90) % 360;
		
		//double k = (speedL > speedR) ? toInches(((0.02 * speedR) * (speedL/speedR - 1)) / toEncoderCounts(l)) :
					//toInches(((0.02 * speedL) * (speedR/speedL - 1)) / toEncoderCounts(l));
		
		double radius = (speedL > speedR) ? (l / (speedL/speedR - 1)) : (l / (speedR/speedL - 1));
		double kT = (speedL > speedR) ? (0.02 * speedR) / (2 * Math.PI * radius) : (0.02 * speedL * 2 * Math.PI) / (2 * Math.PI * radius);
		double alphaPrime = ((90 - parameters[1]) + parameters[2] + 90 + kT) % 360;
		
		/*
		 * If robot is consistently off by around half the chassis length, it's because
		 * currently k is set to either the inner wheels or outer wheels. To fix this problem,
		 * k needs to be configured such that it is between the two. (L/2)
		 */
		
		deltas[0] = Math.sqrt(Math.pow(parameters[0] * Math.sin(Math.toRadians(parameters[1]))
				+ radius * (Math.cos(Math.toRadians(alphaPrime)) - Math.cos(Math.toRadians(alpha))), 2)
				+ Math.pow(parameters[0] * Math.cos(Math.toRadians(parameters[1]))
				+ radius * (Math.sin(Math.toRadians(alphaPrime)) - Math.sin(Math.toRadians(alpha))), 2));
		
		deltas[1] = Math.toDegrees(Math.atan2((parameters[0] * Math.cos(Math.toRadians(parameters[1]))
				+ radius * (Math.sin(Math.toRadians(alphaPrime)) - Math.sin(Math.toRadians(alpha)))),
				parameters[0] * Math.sin(Math.toRadians(parameters[1]))
				+ radius * (Math.cos(Math.toRadians(alphaPrime)) - Math.cos(Math.toRadians(alpha)))));
		
		deltas[2] = (alphaPrime - deltas[1]) % 360;
		
		return deltas;
	}
	
	private double[] newPosition(double parameters[]) {
		
		double[] newPos = new double[3];
    
    int delta = 100; // Can change this later

    ArrayList<double[][]> potential = new ArrayList<double[][]>();
    ArrayList<double[][]> highPotential = new ArrayList<double[][]>();
    
		if (Math.abs(speedL) > minSpeedL && Math.abs(speedR) > minSpeedR) {
			
			for (int i = 0; i < 9; i++) {
				char[] binaryRep = Integer.toBinaryString(i).toCharArray();
				int side = Integer.parseInt(String.valueOf(binaryRep[0])) - 1;
				int up = Integer.parseInt(String.valueOf(binaryRep[1])) - 1;
				
				double[] direction = {speedL + side * delta, speedR + up * delta};
				double[][] temp = {checkPosition(parameters, speedL + side * delta, speedR + up * delta), direction};
				
				potential.add(temp);
			}
			
			for (int i = 0; i < 9; i++) {
				if (potential.get(i)[0][0] < parameters[0] && 
					potential.get(i)[0][1] < parameters[1] &&
					potential.get(i)[0][2] < parameters[2]) {
					highPotential.add(potential.get(i));
				}
			}
			
			Collections.sort(highPotential, new CompareParams());
			Collections.sort(potential, new CompareParams());
			
			int speedL, speedR;
			
			if (highPotential.size() > 0) {
				speedL = (int) highPotential.get(highPotential.size() - 1)[1][0];
				speedR = (int) highPotential.get(highPotential.size() - 1)[1][1];
			} else {
				speedL = (int) potential.get(potential.size() - 1)[1][0];
				speedR = (int) potential.get(potential.size() - 1)[1][1];
			}
			
			Robot.chassisSubsystem.tankMove(speedL/maxSpeedL, speedR/minSpeedL);
		} else {
      
      /*
        These nasty for loops may pose an issue.
      */

      //Top edge

      for (int i = (int)-minSpeedR; i < minSpeedR; i+=delta) {
          double[] direction = {i, minSpeedL};
          double[][] temp = {checkPosition(parameters, i, minSpeedL), direction};
          potential.add(temp);
      }

      //Bottom edge

      for (int i = (int)-minSpeedR; i < minSpeedR; i+=delta) {
        double[] direction = {i, -minSpeedL};
        double[][] temp = {checkPosition(parameters, i, -minSpeedL), direction};
        potential.add(temp);
      }

      //Left edge

      for (int i = (int)minSpeedL; i > -minSpeedL; i-=delta) {
        double[] direction = {-minSpeedR, i};
        double[][] temp = {checkPosition(parameters, -minSpeedR, i), direction};
        potential.add(temp);
      }

      //Left edge

      for (int i = (int)minSpeedL; i > -minSpeedL; i-=delta) {
          double[] direction = {minSpeedR, i};
          double[][] temp = {checkPosition(parameters, minSpeedR, i), direction};
          potential.add(temp);
      }

      for (int i = 0 ; i < potential.size(); i++) {  
          if (potential.get(i)[0][0] < parameters[0] && 
          potential.get(i)[0][1] < parameters[1] &&
          potential.get(i)[0][2] < parameters[2]) {
          highPotential.add(potential.get(i));
        }
      }

      Collections.sort(highPotential, new CompareParams());
			Collections.sort(potential, new CompareParams());
			
			int speedL, speedR;
			
			if (highPotential.size() > 0) {
				speedL = (int) highPotential.get(highPotential.size() - 1)[1][0];
				speedR = (int) highPotential.get(highPotential.size() - 1)[1][1];
			} else {
				speedL = (int) potential.get(potential.size() - 1)[1][0];
				speedR = (int) potential.get(potential.size() - 1)[1][1];
			}
			
			Robot.chassisSubsystem.tankMove(speedL/maxSpeedL, speedR/minSpeedL);
      
		}
		
		return newPos;
	}

    public FollowCommand() {
        requires(Robot.chassisSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	newPosition(parameters); //Need to somehow feed new data to this command
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (parameters[0] <= 10) && (parameters[1] <= 5) && (parameters[2] <= 5);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
