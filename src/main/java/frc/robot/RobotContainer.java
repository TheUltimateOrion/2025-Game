// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutoAim;
import frc.robot.commands.ShootNote;
import frc.robot.commands.SwerveJoystickAuto;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.HookSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(0, 1); // TODO: update IDs?
  // private final HookSubsystem hook = new HookSubsystem(Constants.Hook.leftHook,
  // Constants.Hook.rightHook);

  private final ShooterSubsystem shooter = new ShooterSubsystem(Constants.Shooter.mID, Constants.Shooter.mInverted);
  private final PhotonVision photonVision = new PhotonVision("1");

  private final XboxController controller = new XboxController(0);
  // private final Joystick joystick = new Joystick(0);

  private final ElevatorSystem elevator = new ElevatorSystem(Constants.Elevator.motorLeftID,
      Constants.Elevator.motorRightID);

  public RobotContainer() {
    // create named commands for pathplanner here
    NamedCommands.registerCommand("Drop", new ShootNote(shooter, () -> 0.1));

    swerveSubsystem.setDefaultCommand(new SwerveJoystickAuto(
        swerveSubsystem,
        () -> controller.getLeftY(),
        () -> controller.getLeftX(),
        () -> -controller.getRightY(),
        () -> -controller.getRightX()));

    shooter.setDefaultCommand(new ShootNote(shooter, () -> controller.getRightTriggerAxis()));
    configureBindings();
  }

  // * changed to XboxController.Button.kButton.value
  // * to improve readability in the code.
  // * made all the variables for additional readability
  // * uncomment if needed.

  private final int leftBumper = XboxController.Button.kLeftBumper.value;
  // private final int rightBumper = XboxController.Button.kRightBumper.value;
  private final int aButton = XboxController.Button.kA.value;
  // private final int bButton = XboxController.Button.kB.value;
  private final int xButton = XboxController.Button.kX.value;
  private final int yButton = XboxController.Button.kY.value;
  // private final int backButton = XboxController.Button.kBack.value;
  // private final int startButton = XboxController.Button.kStart.value;
  // private final int leftStickButton = XboxController.Button.kLeftStick.value;
  // private final int rightStickButton = XboxController.Button.kRightStick.value;

  private void configureBindings() {
    // a button -> toggle talons (reversed)
    new JoystickButton(controller, aButton)
        .onTrue(new InstantCommand(() -> intakeSubsystem.toggleMotors(0.5)));

    // y button -> zero heading
    new JoystickButton(controller, yButton)
        .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    // up, down, left -> forward, backward, stop
    new POVButton(controller, 0).whileTrue(new InstantCommand(() -> elevator.setSpeed(Constants.Elevator.motorSpeed)));
    new POVButton(controller, 180)
        .whileTrue(new InstantCommand(() -> elevator.setSpeed(-Constants.Elevator.motorSpeed)));
    new POVButton(controller, 270).onTrue(new InstantCommand(() -> elevator.stop()));

    // left bumper -> swerve joystick
    new JoystickButton(controller, leftBumper).whileTrue(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()));

    // x button -> auto aim
    new JoystickButton(controller, xButton)
        .whileTrue(new AutoAim(swerveSubsystem, photonVision));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Drop It");
  }
}
