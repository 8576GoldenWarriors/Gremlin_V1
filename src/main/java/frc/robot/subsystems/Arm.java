// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  static CANSparkMax armMotor = new CANSparkMax(55, MotorType.kBrushless);
  public Arm() {
    armMotor.setIdleMode(IdleMode.kBrake);
  }
  public static void setSpeed(double speed){
    armMotor.set(speed);
  }
  public static double getEncoderValue(){
    return armMotor.getEncoder().getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
