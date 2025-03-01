// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import java.sql.Driver;
import java.util.List;
// import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAim extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final PhotonVision photonVisionSubsystem;

  /** Creates a new Aim. */
  public AutoAim(SwerveSubsystem swerveSubsystem, PhotonVision photonVisionSubsystem) {
    // Use addRequir ements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;
    addRequirements(swerveSubsystem, photonVisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = photonVisionSubsystem.getResult();
    if (!result.hasTargets()) {
      return;
    }
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();

    int IDP = 0;
    int IDS = 0;
    if (DriverStation.getAlliance().get().toString() == "Blue") {
      IDP = Constants.Photon.blueIDP;
      IDS = Constants.Photon.blueIDS;
    } else {
      IDP = Constants.Photon.redIDP;
      IDS = Constants.Photon.redIDS;
    }

    if (target.getFiducialId() != IDP) {
      int index = searchTarget(targets, IDS);
      if (index == -1) {
        return;
      }

      target = targets.get(index);
    }
  }

  private int searchTarget(List<PhotonTrackedTarget> targets, int ID) {
    for (int i = 0; i < targets.size(); i++) {
      PhotonTrackedTarget target = targets.get(i);
      if (target.getFiducialId() == ID) {
        return i;
      }
    }
    return -1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
