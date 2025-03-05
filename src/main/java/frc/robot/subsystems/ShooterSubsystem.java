// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonSRX talon1;
  private final TalonSRX talon2;

  private boolean isShooting = false;

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem(int motor1Id, int motor2Id) {

    talon1 = new TalonSRX(motor1Id);
    talon2 = new TalonSRX(motor2Id);

    talon2.setInverted(true);

    // MotorOutputConfigs config1 = new MotorOutputConfigs();
    // MotorOutputConfigs config2 = new MotorOutputConfigs();
    // config1.Inverted = m_1Inverted ? InvertedValue.CounterClockwise_Positive :
    // InvertedValue.Clockwise_Positive;
    // config1.Inverted = m_2Inverted ? InvertedValue.CounterClockwise_Positive :
    // InvertedValue.Clockwise_Positive;

    // m_1.getConfigurator().apply(config1);
    // m_2.getConfigurator().apply(config2);

  }

  public void toggleMotors(double speed) {
    isShooting = !isShooting;
    // talon1.set(TalonSRXControlMode.PercentOutput, isShooting ? 0 : speed);
    talon1.set(TalonSRXControlMode.PercentOutput, speed);
    talon2.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stopMotors() {
    isShooting = false;
    talon1.set(TalonSRXControlMode.PercentOutput, 0);
    talon2.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public boolean isRunning() {
    return isShooting;
  }
}
