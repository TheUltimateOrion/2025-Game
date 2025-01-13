// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutoAim;
import frc.robot.commands.HookDPAD;
// import frc.robot.commands.NoteIntake;
import frc.robot.commands.ShootNote;
import frc.robot.commands.SwerveJoystickAuto;
import frc.robot.commands.SwerveJoystickCmd;
// import frc.robot.commands.ReverseShooter;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final HookSubsystem hook = new HookSubsystem(Constants.Hook.leftHook, Constants.Hook.rightHook);
  @SuppressWarnings("unused")
  private final IntakeSubsystem intake = new IntakeSubsystem(Constants.Intake.m_1ID, Constants.Intake.m_1Inverted, Constants.Intake.m_2ID, Constants.Intake.m_2Inverted);
  private final ShooterSubsystem shooter = new ShooterSubsystem(Constants.Shooter.mID, Constants.Shooter.mInverted);
  private final PhotonVision photonVision = new PhotonVision("1");
  
  private final PS4Controller ps4 = new PS4Controller(0);
  // private final Joystick joystick = new Joystick(0);

  public RobotContainer() {

    //create named commands for pathplanner here
      NamedCommands.registerCommand("Drop", new ShootNote(shooter, ()-> 0.1));

      swerveSubsystem.setDefaultCommand(new SwerveJoystickAuto(
      swerveSubsystem, 
      () -> ps4.getLeftY(),
      () -> -ps4.getLeftX(),
      () -> -ps4.getRightY(),
      () -> -ps4.getRightX()));


      shooter.setDefaultCommand(new ShootNote(shooter, () -> ps4.getR2Axis()));
    configureBindings();
  }

  private void configureBindings() {

    new JoystickButton(ps4, 4).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new POVButton(ps4, 0).whileTrue(new HookDPAD(hook, true));
    new POVButton(ps4, 180).whileTrue(new HookDPAD(hook, false));
    new JoystickButton(ps4, 5).whileTrue(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -ps4.getLeftY(),
      () -> -ps4.getLeftX(),
      () -> -ps4.getRightX()
      ));
    new JoystickButton(ps4, 3).whileTrue(new AutoAim(swerveSubsystem, photonVision));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Drop It");
  }
}
