// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.Elevator;
import frc.robot.Constants.Keybindings;
import frc.robot.Constants.Shooter;
import frc.robot.commands.ShootNote;
import frc.robot.commands.SwerveJoystickAuto;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.VisionCmd;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSystem;

public class RobotContainer {
        // subsystems
        private final ElevatorSystem elevator = new ElevatorSystem(Elevator.M_ID_LEFT, Elevator.M_ID_RIGHT);
        private final ShooterSubsystem shooter = new ShooterSubsystem(Shooter.M_ID_LEFT, Shooter.M_ID_RIGHT);
        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final VisionSystem visionSystem = new VisionSystem();
        // controllers
        private final XboxController controller = new XboxController(0);

        // motors
        private final Servo servo = new Servo(0);
        private boolean servoState = false;

        // commands
        public RobotContainer() {

                // create named commands for pathplanner here
                NamedCommands.registerCommand("ShootL2", new ShootNote(shooter, () -> 0.1));

                // swerveSubsystem.setDefaultCommand(new SwerveJoystickAuto(
                // swerveSubsystem,
                // () -> controller.getLeftY(),
                // () -> controller.getLeftX(),
                // () -> -controller.getRightY(),
                // () -> -controller.getRightX()));

                shooter.setDefaultCommand(new ShootNote(shooter,
                                () -> controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
                configureBindings();
        }

        private void configureBindings() {
                new JoystickButton(controller, Keybindings.BUTTON_A)
                                .onTrue(new InstantCommand(() -> servoState = !servoState))
                                .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                        servo.setAngle(servoState ? 270 : -270);
                                })));

                // zero heading
                new JoystickButton(controller, Keybindings.BUTTON_Y)
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                // configure DPAD
                // elevator forward, backward -> start and stop
                new POVButton(controller, Keybindings.DPAD_UP)
                                .whileTrue(
                                                new RepeatCommand(new InstantCommand(() -> elevator
                                                                .move(ElevatorSystem.Direction.Up))))
                                .onFalse(new InstantCommand(() -> elevator.move(ElevatorSystem.Direction.Stop)));
                new POVButton(controller, Keybindings.DPAD_DOWN)
                                .whileTrue(
                                                new RepeatCommand(new InstantCommand(() -> elevator
                                                                .move(ElevatorSystem.Direction.Down))))
                                .onFalse(new InstantCommand(() -> elevator.move(ElevatorSystem.Direction.Stop)));

                // left bumper -> swerve joystick
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> controller.getLeftY(),
                                () -> controller.getLeftX(),
                                () -> -controller.getRightX()));
                // new JoystickButton(controller, Keybindings.BUMPER_LEFT).whileTrue(new
                // SwerveJoystickCmd(
                // swerveSubsystem,
                // () -> controller.getLeftY(),
                // () -> controller.getLeftX(),
                // () -> -controller.getRightX()));
                visionSystem.setDefaultCommand(new VisionCmd(visionSystem));
        }

        public Command getAutonomousCommand() {
                return new PathPlannerAuto("Coral 1");
        }
}
