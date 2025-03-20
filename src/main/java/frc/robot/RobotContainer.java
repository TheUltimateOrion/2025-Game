// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

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
        private final XboxController movementController = new XboxController(0);
        private final XboxController coralController = new XboxController(1);

        // motors
        private final Servo servo = new Servo(0);
        private boolean servoState = false;
        Command visionCommand;

        // commands
        public RobotContainer() {
                // create named commands for pathplanner here
                NamedCommands.registerCommand("Shoot", new ShootNote(shooter, () -> 1.));
                NamedCommands.registerCommand("LimelightSearch", new VisionCmd(visionSystem, swerveSubsystem));

                shooter.setDefaultCommand(new ShootNote(shooter,
                                () -> coralController.getRightTriggerAxis() - coralController.getLeftTriggerAxis()));

                this.visionCommand = new VisionCmd(visionSystem, swerveSubsystem);
                configureBindings();
        }

        private void configureBindings() {
                new JoystickButton(coralController, Keybindings.BUTTON_A)
                                .onTrue(new InstantCommand(() -> servoState = !servoState))
                                .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                        servo.setAngle(servoState ? 270 : -270);
                                })));

                // zero heading
                new JoystickButton(coralController, Keybindings.BUTTON_Y)
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                // configure DPAD
                // elevator forward, backward -> start and stop
                new POVButton(coralController, Keybindings.DPAD_UP)
                                .whileTrue(
                                                new RepeatCommand(new InstantCommand(() -> elevator
                                                                .move(ElevatorSystem.Direction.Up))))
                                .onFalse(new InstantCommand(() -> elevator.move(ElevatorSystem.Direction.Stop)));
                new POVButton(coralController, Keybindings.DPAD_DOWN)
                                .whileTrue(
                                                new RepeatCommand(new InstantCommand(() -> elevator
                                                                .move(ElevatorSystem.Direction.Down))))
                                .onFalse(new InstantCommand(() -> elevator.move(ElevatorSystem.Direction.Stop)));

                // left bumper -> swerve joystick
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> movementController.getLeftY(),
                                () -> movementController.getLeftX(),
                                () -> -movementController.getRightX()));
        }

        public Command getAutonomousCommand() {
                // return new InstantCommand(() -> {
                // });
                return visionCommand;
                // return new InstantCommand(() -> {
                // Timer.delay(5);
                // swerveSubsystem.zeroHeading();
                // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.1, 0, 0,
                // swerveSubsystem.getRotation2d());
                // swerveSubsystem.drive(speeds, true);
                // Timer.delay(1);
                // swerveSubsystem.stopModules();
                // });
                // return new PathPlannerAuto("Coral Auto");
        }
}
