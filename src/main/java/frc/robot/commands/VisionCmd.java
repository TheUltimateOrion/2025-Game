package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSystem;

@SuppressWarnings("unused")
public class VisionCmd extends Command {
  private final VisionSystem visionSystem;
  private final SwerveSubsystem swerveSubsystem;

  private boolean shouldFinish = false;

  public VisionCmd(VisionSystem visionSystem, SwerveSubsystem swerveSubsystem) {
    addRequirements(visionSystem, swerveSubsystem);
    this.visionSystem = visionSystem;
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // boolean cond1 = LimelightHelpers.getTA("limelight") > 10;
    // boolean cond2 = Math.abs(LimelightHelpers.getTX("limelight")) < 45;
    // if ((cond1 && cond2) || !LimelightHelpers.getTV("limelight")) {
    // this.shouldFinish = true;
    // swerveSubsystem.stopModules();
    // }
    drive();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return this.shouldFinish;
  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is
  // proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional
  // to the
  // "tx" value from the Limelight.
  double limelight_aim_proportional() {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our
    // proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .05;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
    // rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Constants.DriveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RPS;

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are
  // different.
  // if your limelight and target are mounted at the same or similar heights, use
  // "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTA("limelight") * kP;
    targetingForwardSpeed *= Constants.DriveConstants.TELE_DRIVE_MAX_SPEED_MPS;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private void drive() {

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    final var rot_limelight = limelight_aim_proportional();
    final var forward_limelight = limelight_range_proportional();

    ChassisSpeeds speeds = new ChassisSpeeds(forward_limelight, 0.0, rot_limelight);
    swerveSubsystem.drive(speeds, false);
  }

}
