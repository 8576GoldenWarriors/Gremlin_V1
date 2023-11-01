// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveWithJoystickCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  //private JoystickButton intake1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithJoystickCommand(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.]
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Starting joystick drive command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = RobotContainer.joystickDrive.getRawAxis(1);// * Math.abs(RobotContainer.joystick.getRawAxis(1));
    double turningSpeed = RobotContainer.joystickDrive.getRawAxis(4);
    
    
    
    // * Math.abs(RobotContainer.joystick.getRawAxis(4));
    Drivetrain.setSpeed(forwardSpeed * 0.95, (turningSpeed * 0.7));

    // if(RobotContainer.joystick.getRawButtonPressed  (4)){
    //   Intake.intake(-0.6);
    // }
    // if(RobotContainer.joystick.getRawButtonPressed(3)){
    //   Intake.intake(-0);
    // }


    /*else{
      Robot.intake.set(0);
    }

    if(RobotContainer.joystick.getRawButtonPressed(6)){
      Robot.intake.set(-0.5);
    }
    else{
      Robot.intake.set(0);
    }*/

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}