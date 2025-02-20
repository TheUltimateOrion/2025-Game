package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DCMotor extends SubsystemBase {
  private final TalonFX motor;
  private static final double DEFAULT_SPEED = 0.25;

  public DCMotor(int motorID) {
    motor = new TalonFX(motorID);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.set(0);
  }

  public void addSpeed() {
    double speed = motor.get();
    if (speed >= 1) {
      return;
    }

    motor.set(speed + DEFAULT_SPEED);
  }

  public void subtractSpeed() {
    double speed = motor.get();
    if (speed <= -1) {
      return;
    }

    motor.set(speed - DEFAULT_SPEED);
  }
}