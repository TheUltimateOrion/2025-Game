// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
