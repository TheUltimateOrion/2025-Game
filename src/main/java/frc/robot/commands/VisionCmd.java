package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.VisionSystem;

@SuppressWarnings("unused")
public class VisionCmd extends Command {

  public VisionCmd(VisionSystem visionSystem) {
    addRequirements(visionSystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // System.out.println(LimelightHelpers.getTX(""));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
