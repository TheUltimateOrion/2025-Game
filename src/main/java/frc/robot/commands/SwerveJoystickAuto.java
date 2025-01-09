  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotStructure;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickAuto extends Command {



  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, xTurnFunction, yTurnFunction;
  private final SlewRateLimiter xSpdLimiter, ySpdLimiter, turnSpdLimiter;
  private final PIDController turningPID;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickAuto(SwerveSubsystem swerveSubsystem, 
                           Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> xTurnFunction, Supplier<Double> yTurnFunction) {

    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.xTurnFunction = xTurnFunction;
    this.yTurnFunction = yTurnFunction;

    this.xSpdLimiter= new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUPS);
    this.ySpdLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUPS);
    this.turnSpdLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUPS);

    turningPID = new PIDController(0.1, 0, 0);
    turningPID.enableContinuousInput(-180, 180);
  
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}



  
  @Override
  public void execute() {

    double xSpd = xSpdFunction.get();
    double ySpd = ySpdFunction.get();

    double xTurn = xTurnFunction.get();
    double yTurn = yTurnFunction.get();

    double angle = Units.radiansToDegrees(Math.atan2(yTurn, xTurn));
    double turnSpd = turningPID.calculate(swerveSubsystem.getHeading(), angle);


    //deadband
    if (Math.abs(xSpd) <= Constants.DriveConstants.kJoystickDeadband){
        xSpd = 0;
    }
    if (Math.abs(ySpd) <= Constants.DriveConstants.kJoystickDeadband){
      ySpd = 0;
    }
    if (Math.abs(xTurn) <= Constants.DriveConstants.kJoystickDeadband){
      if (Math.abs(yTurn) <= Constants.DriveConstants.kJoystickDeadband){
        turnSpd = 0;
      }
    }
    

    xSpd = xSpdLimiter.calculate(xSpd) * Constants.DriveConstants.kTeleDriveMaxSpeedMPS;
    ySpd = ySpdLimiter.calculate(ySpd) * Constants.DriveConstants.kTeleDriveMaxSpeedMPS;
    turnSpd = turnSpdLimiter.calculate(turnSpd) * Constants.DriveConstants.kTeleDriveMaxSpeedMPS;

    //make Chassis speeds
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpd, ySpd, turnSpd, swerveSubsystem.getRotation2d());
    

    //convert chassis speeds to module states
    SwerveModuleState[] moduleStates = RobotStructure.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //output states to modules
    swerveSubsystem.setModuleStates(moduleStates);


    SwerveModuleState[] states = swerveSubsystem.getModuleStates();
    SmartDashboard.putString("1", states[0].toString());
    SmartDashboard.putString("2", states[1].toString());
    SmartDashboard.putString("3", states[2].toString());
    SmartDashboard.putString("4", states[3].toString());
    SmartDashboard.putNumber("target Angle", angle);
    SmartDashboard.putNumber("turn speed", turnSpd);
    SmartDashboard.putNumber("x speed", xSpd);
    SmartDashboard.putNumber("y speed", ySpd);
    SmartDashboard.putNumber("xTurn", xTurn);
    SmartDashboard.putNumber("yTurn", yTurn);

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
