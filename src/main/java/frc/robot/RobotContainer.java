// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.DeepCageSystem;
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
        private final DeepCageSystem deepCage = new DeepCageSystem();

        // controllers
        private final XboxController movementController = new XboxController(0);
        private final XboxController coralController = new XboxController(1);

        // motors
        private final Servo servo = new Servo(9);
        private boolean servoState = false;
        Command visionCommand;

        // commands
        public RobotContainer() {
                // create named commands for pathplanner here

                shooter.setDefaultCommand(new ShootNote(shooter,
                                () -> coralController.getRightTriggerAxis() - coralController.getLeftTriggerAxis()));

                this.visionCommand = new VisionCmd(visionSystem, swerveSubsystem);
                configureBindings();
        }

        private void configureBindings() {
                new JoystickButton(coralController, Keybindings.BUTTON_A)
                                .onTrue(new InstantCommand(() -> servoState = !servoState))
                                .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                        servo.setAngle(servoState ? 0 : 180);
                                })));

                // zero heading
                new JoystickButton(coralController, Keybindings.BUTTON_Y)
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                new JoystickButton(coralController, Keybindings.BUMPER_RIGHT)
                                .whileTrue(new RepeatCommand(new InstantCommand(
                                                () -> deepCage.setSpeed(Constants.DeepCage.MOTOR_SPEED),
                                                deepCage)))
                                .onFalse(new InstantCommand(() -> deepCage.setSpeed(0), deepCage));
                new JoystickButton(coralController, Keybindings.BUMPER_LEFT)
                                .whileTrue(new RepeatCommand(new InstantCommand(
                                                () -> deepCage.setSpeed(-Constants.DeepCage.MOTOR_SPEED),
                                                deepCage)))
                                .onFalse(new InstantCommand(() -> deepCage.setSpeed(0), deepCage));
                new POVButton(coralController, Keybindings.DPAD_UP)
                                .whileTrue(new InstantCommand(() -> elevator.set(Elevator.L1)));
                new POVButton(coralController, Keybindings.DPAD_RIGHT)
                                .whileTrue(new InstantCommand(() -> elevator.set(Elevator.L2)));
                new POVButton(coralController, Keybindings.DPAD_DOWN)
                                .whileTrue(new InstantCommand(() -> elevator.set(Elevator.L3)));
                //new POVButton(coralController, Keybindings.DPAD_LEFT)
                //                .whileTrue(new InstantCommand(() -> elevator.set(Elevator.L4)));

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> movementController.getLeftY(),
                                () -> movementController.getLeftX(),
                                () -> movementController.getRightX()));
        }

        public Command getAutonomousCommand() {
                // return new InstantCommand(() -> {
                // });
                return new PathPlannerAuto("New Auto");
                // Pose2d target = new Pose2d(5.095, 2.644, Rotation2d.fromDegrees(-60));
                // // Create the constraints to use while pathfinding
                // PathConstraints constraints = new PathConstraints(
                // 0.5, 1.0,
                // Units.degreesToRadians(540), Units.degreesToRadians(360));

                // // Since AutoBuilder is configured, we can use it to build pathfinding
                // commands
                // Command pathfindingCommand = AutoBuilder.pathfindToPose(
                // target,
                // constraints,
                // 0.0 // Goal end velocity in meters/sec
                // ).andThen(new InstantCommand(() -> elevator.set(Elevator.L2))
                // .andThen(new ShootNote(shooter, () -> 1.)));
                // return pathfindingCommand;
                // return new InstantCommand(() -> {
                // Timer.delay(2);
                // swerveSubsystem.zeroHeading();
                // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.1, 0, 0,
                // swerveSubsystem.getPose().getRotation());
                // swerveSubsystem.drive(speeds, true);
                // Timer.delay(1);
                // swerveSubsystem.stopModules();
                // });
                // return new PathPlannerAuto("Coral Auto");
        }
}
