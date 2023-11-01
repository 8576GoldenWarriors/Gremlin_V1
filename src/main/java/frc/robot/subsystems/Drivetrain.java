// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  static CANSparkMax leftTop = new CANSparkMax(51,MotorType.kBrushed );
  static CANSparkMax leftBottom = new CANSparkMax(52,MotorType.kBrushed );
  static CANSparkMax rightTop = new CANSparkMax(53,MotorType.kBrushed );
  static CANSparkMax rightBottom = new CANSparkMax(54,MotorType.kBrushed );

  
  //static RelativeEncoder leftEncoder = leftTop.getEncoder();
  //static RelativeEncoder rightEncoder = rightTop.getEncoder();


  static MotorControllerGroup leftSide = new MotorControllerGroup(leftBottom, leftTop);
  static MotorControllerGroup rightSide = new MotorControllerGroup(rightBottom, rightTop);
  public static DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);

  
  
  public Drivetrain() {
    /*leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    // zeros out the motors and encoders - J

    rightEncoder.setPositionConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor / 60);
*/
    leftSide.setInverted(true);
    rightSide.setInverted(false);
  }

  public static void setSpeed(double speed, double turn){
    drive.arcadeDrive(speed, -turn);
  }

  public static void setDrive(double speed){
    leftBottom.set(-speed);
    leftTop.set(-speed);
    rightBottom.set(speed);
    rightTop.set(speed);
  }

  public static void TankDrive(double speed1, double speed2){
    drive.tankDrive(speed1, -speed2);
  }

  /*public static double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  public static double getLeftEncoderPosition() {
    return -leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return -leftEncoder.getVelocity();
  }

  public static double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
