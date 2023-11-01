// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PID_Arm extends CommandBase {
  /** Creates a new PID_Arm. */
  public Arm arm;

  public double setpoint;
  public double currentPosition;
  public double error;
  public double lastError;
  public double kP;
  public double kD;
  public double derivative;
  public double motorPower;

  public boolean isDone;

  public PID_Arm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;

    //Constants
    kP = 0.4; //Change as neccessary 
    kD = 0.05; //Change as neccessary

    setpoint = 2; //CHANGE THIS LATER TO GET END POINT
    currentPosition = Arm.getEncoderValue();
    error = setpoint; 
    lastError = setpoint;

    isDone = false;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    if(Math.abs(error) > (Math.abs(setpoint) * 0.1)){
      error = setpoint-currentPosition;
      derivative = error - lastError;

      motorPower = (kP*error)+(kD*derivative);
      
      Arm.setSpeed(motorPower);    
      
      lastError = error;
    }
    else{
      isDone = true;
      end(isDone);
    } //Change 1.5 if necessary
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.setSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return isDone;

  }
}
