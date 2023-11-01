// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  static final VictorSPX leftMotor = new VictorSPX(1);
  static final VictorSPX rightMotor = new VictorSPX(2);
  /** Creates a new Claw. */
  public Claw() {
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

  }

  public static void setSpeed(double speed){
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, (-speed * 0.75));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
