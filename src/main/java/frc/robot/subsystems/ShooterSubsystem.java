// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX motor;

  /** Creates a new ShooterSubsystem. */
  @SuppressWarnings("removal")
  public ShooterSubsystem(int ID, boolean Inverted) {

    motor = new TalonFX(ID, "rio");
    motor.setInverted(Inverted);

    // MotorOutputConfigs config = new MotorOutputConfigs();
    // config.Inverted = Inverted ? InvertedValue.CounterClockwise_Positive :
    // InvertedValue.Clockwise_Positive;

    // motor.getConfigurator().apply(config);

  }

  public void setMotors(double speed) {
    motor.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (motor.getVelocity().getValueAsDouble() > 0) {
      SmartDashboard.putString("Shooter Status", "Running");
    } else {
      SmartDashboard.putString("Shooter Status", "Idle");
    }
  }
}
