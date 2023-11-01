// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PID_Drive extends CommandBase {
  /** Creates a new PID_Drive. */
  public double setpoint;
  public double currentPosition;
  public double error;
  public double lastError;
  public double kP;
  public double kI;
  public double kD;
  public double integral;
  public double derivative;
  public double motorPower;
  public boolean isDone;
  
  public PID_Drive(Drivetrain drivetrain, double setpoint) {

    this.setpoint = setpoint;
    currentPosition = 0;
    lastError = setpoint;
    kP = 0.5;//CHANGE CONSTANTS AS NEEDED
    kI = 0.1;
    kD = 0.1;
    integral = 0;
    derivative = 0;
    isDone = false;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*error = setpoint - currentPosition;
    derivative = error - lastError;

    if(error<setpoint*0.1)
    motorPower = (kP*error)+(kI*integral)*(kD*derivative);
    lastError = error;*/
    if (Math.abs(error) > (Math.abs(setpoint) / 2)){

      error = Math.abs(setpoint) - Math.abs(1);
      integral = integral + error;
      derivative = error - lastError;
      motorPower = (kP * error) + (kI * integral) + (kD * derivative);
      
      if (motorPower > 0.65){
        motorPower = 0.65;
      }

      if (motorPower < .20){
        motorPower = 0;
      }
      
      if (setpoint < 0){
        motorPower = -motorPower;
      }

      if (motorPower == 0){
        isDone = true;
      }

      System.out.println(error);
      
      //DrivetrainSubsystem.arcadeDrive((-motorPower), 0);
      Drivetrain.TankDrive((-motorPower - (motorPower / 50)), (-motorPower));

    }
    else{
      //System.out.println("Done");
      isDone = true;
      //end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
