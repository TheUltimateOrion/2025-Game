package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSystem extends SubsystemBase {
  private final TalonSRX motor;
  private static final double DEFAULT_SPEED = 0.25;

  // TODO: add the real IDs if required
  private final DigitalInput toplimitSwitch = new DigitalInput(0);
  private final DigitalInput bottomlimitSwitch = new DigitalInput(1);

  public ElevatorSystem(int motorID) {
    motor = new TalonSRX(motorID);
  }

  public void setSpeed(double speed) {
    motor.set(ControlMode.Velocity, speed);

    if (toplimitSwitch.get() && speed > 0 || bottomlimitSwitch.get() && speed < 0) {
      stop();
    }
  }

  public void stop() {
    motor.set(ControlMode.Velocity, 0);
  }

  public void addSpeed() {
    double speed = motor.getSelectedSensorVelocity();
    if (speed >= 1) {
      return;
    }

    motor.set(ControlMode.Velocity, speed + DEFAULT_SPEED);
  }

  public void subtractSpeed() {
    double speed = motor.getSelectedSensorVelocity();
    if (speed <= -1) {
      return;
    }

    motor.set(ControlMode.Velocity, speed - DEFAULT_SPEED);
  }
}