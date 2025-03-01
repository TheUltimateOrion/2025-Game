// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HookSubsystem;

public class HookDPAD extends Command {

  private final HookSubsystem hook;
  private final boolean up;

  /** Creates a new HookDPAD. */
  public HookDPAD(HookSubsystem hook, boolean up) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hook = hook;
    this.up = up;

    addRequirements(hook);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double position = hook.getLeftPosition();

    if (position <= Constants.Hook.max && up) {
      hook.setMotors(0);
      System.out.println("too High");
      return;
    }
    if (position >= Constants.Hook.min && !up) {
      System.out.println("too low");
      hook.setMotors(0);
      return;
    }

    double speed = 0;
    if (up) {
      speed = Constants.Hook.upSpeed;
    } else {
      speed = Constants.Hook.downSpeed;
    }
    hook.setMotors(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    hook.setMotors(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
