  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turnSpdFunction;
  private final SlewRateLimiter xSpdLimiter, ySpdLimiter, turnSpdLimiter;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, 
                           Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turnSpdFunction) {

    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turnSpdFunction = turnSpdFunction;

    this.xSpdLimiter= new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUPS);
    this.ySpdLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUPS);
    this.turnSpdLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUPS);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

StructPublisher<ChassisSpeeds> chassisSpeed = NetworkTableInstance.getDefault()
.getStructTopic("Chassis Speeds", ChassisSpeeds.struct).publish();
  
  @Override
  public void execute() {

    double xSpd = xSpdFunction.get();
    double ySpd = ySpdFunction.get();
    double turnSpd = turnSpdFunction.get();

    //deadband
    if (Math.abs(xSpd) <= Constants.DriveConstants.kJoystickDeadband){
        xSpd = 0;
    }
    if (Math.abs(ySpd) <= Constants.DriveConstants.kJoystickDeadband){
      ySpd = 0;
    }
    if (Math.abs(turnSpd) <= Constants.DriveConstants.kJoystickDeadband){
      turnSpd = 0;
    }
    // SmartDashboard.putNumber("turn speed", turnSpd);
    // SmartDashboard.putNumber("x speed", xSpd);

    // SmartDashboard.putNumber("y speed", ySpd);
    xSpd = xSpdLimiter.calculate(xSpd) * Constants.DriveConstants.kTeleDriveMaxSpeedMPS * 0.01;
    ySpd = ySpdLimiter.calculate(ySpd) * Constants.DriveConstants.kTeleDriveMaxSpeedMPS * 0.01;
    turnSpd = turnSpdLimiter.calculate(turnSpd) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRPS *0.05;
    turnSpd = Math.min(Math.PI/32, Math.max(turnSpd, -Math.PI/32));

    //make Chassis speeds
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpd, ySpd, turnSpd, swerveSubsystem.getRotation2d());
    chassisSpeed.set(chassisSpeeds);

    swerveSubsystem.drive(chassisSpeeds);


    // SwerveModuleState[] states = swerveSubsystem.getModuleStates();
    // SmartDashboard.putString("1", states[0].toString());
    // SmartDashboard.putString("2", states[1].toString());
    // SmartDashboard.putString("3", states[2].toString());
    // SmartDashboard.putString("4", states[3].toString());
    

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
