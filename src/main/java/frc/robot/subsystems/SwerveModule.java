// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  private final PIDController turningPID;

  private final CANcoder absoluteEncoder;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderID) {

    absoluteEncoder = new CANcoder(absoluteEncoderID);

    driveMotor = new TalonFX(driveMotorID);
    turnMotor = new TalonFX(turnMotorID);

    // driveMotor.setInverted(driveMotorReversed);
    // turnMotor.setInverted(turningMotorReversed);

    MotorOutputConfigs driveMotorConfig = new MotorOutputConfigs();
    MotorOutputConfigs turnMotorConfig = new MotorOutputConfigs();
    driveMotorConfig.Inverted = driveMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    turnMotorConfig.Inverted = turningMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    driveMotor.getConfigurator().apply(driveMotorConfig);
    driveMotor.getConfigurator().apply(turnMotorConfig);

    turningPID = new PIDController(Constants.SwerveModule.kPturn, 0, Constants.SwerveModule.kDturn);
    turningPID.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }




  public double getDrivePosition(){
    return driveMotor.getRotorPosition().getValueAsDouble() * Constants.SwerveModule.kDriveEncoderRotation2Meter;
  }

  public double getTurnPosition(){
    return turnMotor.getRotorPosition().getValueAsDouble() * Constants.SwerveModule.kTurnEncoderRotation2Rad;

  }

  public double getDriveVelocity(){
    return driveMotor.getRotorVelocity().getValueAsDouble() * Constants.SwerveModule.kDriveEncoderRPS2MPS;
  }

  public double getTurnVelocity(){
    return turnMotor.getRotorVelocity().getValueAsDouble() * Constants.SwerveModule.kTurnEncoderRPS2MPS;

  }

  public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle *= 2 * Math.PI;

    return angle;
  }

  public void resetEncoders(){
    driveMotor.setPosition(0);
    turnMotor.setPosition(getAbsoluteEncoderRad() / Constants.SwerveModule.kTurnEncoderRotation2Rad);
  }


  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    
    if (Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    
    // state = SwerveModuleState.optimize(state, getState().angle);
    state.optimize(getState().angle);

    driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMPS);
    turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
  }

  public void stop(){
    driveMotor.set(0);
    turnMotor.set(0);
  }

  private Rotation2d getAngle(){
    return Rotation2d.fromRadians(getTurnPosition());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
            getDrivePosition(), 
            getAngle());
  }
}

