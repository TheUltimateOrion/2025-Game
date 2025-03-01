// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  private final PIDController turningPID;

  private final CANcoder absoluteEncoder;

  private final DCMotorSim m_driveMotorSimModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(1), 0.001,
          Constants.SwerveModule.kDriveMotorGearRatio),
      DCMotor.getFalcon500Foc(1));

  private final DCMotorSim m_turnMotorSimModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.SwerveModule.kTurnMotorGearRatio),
      DCMotor.getKrakenX60Foc(1));

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderID, double absoluteEncoderOff) {
    absoluteEncoder = new CANcoder(absoluteEncoderID, "rio");
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(absoluteEncoderOff));
    absoluteEncoder.getConfigurator().apply(cc_cfg);

    driveMotor = new TalonFX(driveMotorID, "rio");
    turnMotor = new TalonFX(turnMotorID, "rio");

    driveMotor.getConfigurator().apply(new MotorOutputConfigs()
        .withInverted(driveMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive));
    turnMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(
        turningMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive));

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration().withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(1.0)
        .withRotorToSensorRatio(12.8));

    turnMotor.getConfigurator().apply(fx_cfg);

    turningPID = new PIDController(Constants.SwerveModule.kPturn, 0, Constants.SwerveModule.kDturn);
    turningPID.enableContinuousInput(-Math.PI, Math.PI);
    resetEncoders();
  }

  public double getDrivePosition() {
    return driveMotor.getRotorPosition().getValueAsDouble() * Constants.SwerveModule.kDriveEncoderRotation2Meter;
  }

  public double getTurnPosition() {
    return turnMotor.getRotorPosition().getValueAsDouble() * Constants.SwerveModule.kTurnEncoderRotation2Rad;

  }

  public double getDriveVelocity() {
    return driveMotor.getRotorVelocity().getValueAsDouble() * Constants.SwerveModule.kDriveEncoderRPS2MPS;
  }

  public double getTurnVelocity() {
    return turnMotor.getRotorVelocity().getValueAsDouble() * Constants.SwerveModule.kTurnEncoderRPS2MPS;

  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle *= 2 * Math.PI;

    return angle;
  }

  public void resetEncoders() {
    driveMotor.setPosition(0);
    turnMotor.setPosition(getAbsoluteEncoderRad() / Constants.SwerveModule.kTurnEncoderRotation2Rad);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    // state = SwerveModuleState.optimize(state, getState().angle);
    state.optimize(getState().angle);
    driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMPS);
    if (absoluteEncoder.getDeviceID() == 1) {
      SmartDashboard.putNumber("Current turn", getTurnPosition());
      SmartDashboard.putNumber("Next turn", state.angle.getRadians());

    }
    turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
  }

  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRadians(getTurnPosition());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(),
        getAngle());
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimState driveMotorSim = driveMotor.getSimState();
    TalonFXSimState turnMotorSim = turnMotor.getSimState();

    driveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    turnMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var driveMotorVoltage = driveMotorSim.getMotorVoltageMeasure();
    var turnMotorVoltage = turnMotorSim.getMotorVoltageMeasure();

    m_driveMotorSimModel.setInputVoltage(driveMotorVoltage.in(Volts));
    m_driveMotorSimModel.update(0.020); // assume 20 ms loop time
    m_turnMotorSimModel.setInputVoltage(turnMotorVoltage.in(Volts));
    m_turnMotorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    driveMotorSim.setRawRotorPosition(
        Constants.SwerveModule.kDriveMotorGearRatio * m_driveMotorSimModel.getAngularPositionRotations());
    driveMotorSim.setRotorVelocity(Constants.SwerveModule.kDriveMotorGearRatio
        * Units.radiansToRotations(m_driveMotorSimModel.getAngularVelocityRadPerSec()));
    turnMotorSim.setRawRotorPosition(
        Constants.SwerveModule.kTurnMotorGearRatio * m_turnMotorSimModel.getAngularPositionRotations());
    turnMotorSim.setRotorVelocity(Constants.SwerveModule.kTurnMotorGearRatio
        * Units.radiansToRotations(m_turnMotorSimModel.getAngularVelocityRadPerSec()));
  }
}
