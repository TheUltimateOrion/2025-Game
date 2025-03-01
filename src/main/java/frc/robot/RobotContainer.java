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

import frc.robot.Constants.Elevator;
import frc.robot.Constants.Keybindings;
import frc.robot.Constants.Shooter;
import frc.robot.commands.AutoAim;
import frc.robot.commands.ShootNote;
import frc.robot.commands.SwerveJoystickAuto;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // subsystems
  private final ElevatorSystem elevator = new ElevatorSystem(Elevator.motorLeftID,
      Elevator.motorRightID);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(0, 1);
  private final PhotonVision photonVision = new PhotonVision("1");
  private final ShooterSubsystem shooter = new ShooterSubsystem(Shooter.mID, Shooter.mInverted);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // private final HookSubsystem hook = new HookSubsystem(Constants.Hook.leftHook,
  // Constants.Hook.rightHook);

  // controllers
  private final XboxController controller = new XboxController(0);

  // commands
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

  // keybindings
  private void configureBindings() {
    // toggle intake motors
    new JoystickButton(controller, Keybindings.BUTTON_A)
        .onTrue(new InstantCommand(() -> intakeSubsystem.toggleMotors(0.5)));

    // auto aim
    new JoystickButton(controller, Keybindings.BUTTON_X)
        .whileTrue(new AutoAim(swerveSubsystem, photonVision));

    // zero heading
    new JoystickButton(controller, Keybindings.BUTTON_Y)
        .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    // elevator forward, backward -> start and stop
    new POVButton(controller, Keybindings.DPAD_UP)
        .onTrue(new InstantCommand(() -> elevator.setSpeed(Elevator.motorSpeed)));
    new POVButton(controller, Keybindings.DPAD_DOWN)
        .onTrue(new InstantCommand(() -> elevator.setSpeed(-Elevator.motorSpeed)));

    new POVButton(controller, Keybindings.DPAD_UP)
        .onFalse(new InstantCommand(() -> elevator.stop()));
    new POVButton(controller, Keybindings.DPAD_DOWN)
        .onFalse(new InstantCommand(() -> elevator.stop()));
    // new POVButton(controller, Keybindings.DPAD_LEFT).onTrue(new InstantCommand(()
    // -> elevator.stop()));

    // left bumper -> swerve joystick
    new JoystickButton(controller, Keybindings.BUMPER_LEFT).whileTrue(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Drop It");
  }
}
