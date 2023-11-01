// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveWithJoystickCommand;
import frc.robot.commands.MoveArmUp;
import frc.robot.commands.MoveArmDown;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final DriveWithJoystickCommand djc = new DriveWithJoystickCommand(m_drivetrain);

  public static final Arm m_arm = new Arm();
  //private final MoveArm mac = new MoveArm(m_arm);

  public final static Claw m_claw = new Claw();
  //private final ToggleClaw tcc = new ToggleClaw(m_claw);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final Joystick joystickDrive = new Joystick(0);
  public static final Joystick joystickOp = new Joystick(1);

  public JoystickButton ArmUp = new JoystickButton(joystickOp, 6);
  public JoystickButton ArmDown = new JoystickButton(joystickOp, 5);

  public JoystickButton ClawIn = new JoystickButton(joystickOp, 1);
  public JoystickButton ClawOut = new JoystickButton(joystickOp, 2);  
  public JoystickButton ClawOutMid = new JoystickButton(joystickOp, 4);
  public JoystickButton ClawOutihigh = new JoystickButton(joystickOp, 3);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(djc);
    //m_arm.setDefaultCommand(mac);
    //m_claw.setDefaultCommand(tcc);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    ArmUp.whileTrue(new MoveArmUp(m_arm, -0.5));
    ArmDown.whileTrue(new MoveArmDown(m_arm, 0.5));

    ClawIn.whileTrue(new ToggleClaw(m_claw, 0.2));
    ClawOut.whileTrue(new ToggleClaw(m_claw, -0.2));
    ClawOutMid.whileTrue(new ToggleClaw(m_claw, -0.75));
    ClawOutihigh.whileTrue(new ToggleClaw(m_claw, -0.65));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup(
     new ParallelRaceGroup(
        new RunCommand(
          () -> m_arm.setSpeed(0.2),
          m_arm
        ),
        new WaitCommand(1.75)
      ),
      new ParallelRaceGroup(
        new RunCommand(
          () -> m_arm.setSpeed(0),
          m_arm
        ),
        new RunCommand(
          () -> m_claw.setSpeed(-0.6), 
          m_claw
        ),
        new WaitCommand(0.75)
      ),
      new ParallelRaceGroup(
        new RunCommand(
          () -> m_claw.setSpeed(0), 
          m_claw
        ),
        new RunCommand(
          () -> m_drivetrain.setDrive(-0.5),
          m_drivetrain
        ),
        new WaitCommand(2.2)
      ),
      new RunCommand(
        () -> m_drivetrain.setDrive(0),
        m_drivetrain
        )
     );
    // return null;
  }
  public Drivetrain returnDrivetrain(){
    return m_drivetrain;
  }
  public Arm returnArm(){
    return m_arm;
  }
}
