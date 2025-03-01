// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSubsystem extends SubsystemBase {

  private final TalonFX m_left;
  private final TalonFX m_right;

  /** Creates a new Hook. */
  public HookSubsystem(int leftID, int rightID) {

    m_left = new TalonFX(leftID, "rio");
    m_right = new TalonFX(rightID, "rio");

    resetEncoders();

  }

  public void setMotors(double speed) {

    m_left.set(speed);
    m_right.set(speed);
  }

  public void resetEncoders() {
    m_left.setPosition(0);
    m_right.setPosition(0);
  }

  public double getLeftPosition() {
    return m_left.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hook position", getLeftPosition());
  }
}
