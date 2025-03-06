package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSystem extends SubsystemBase {
  private final TalonFX left;
  private final TalonFX right;
  private static final double DEFAULT_SPEED = 0.25;

  // // TODO: update IDs
  private final DigitalInput toplimitSwitch = new DigitalInput(5);
  private final DigitalInput bottomlimitSwitch = new DigitalInput(4);

  public ElevatorSystem(int leftID, int rightID) {
    left = new TalonFX(leftID);
    right = new TalonFX(rightID);

    left.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
  }

  public void lock() {
    left.set(Elevator.ANTI_GRAVITY);
    right.set(Elevator.ANTI_GRAVITY);
  }

  public void setSpeed(double speed) {
    if (!toplimitSwitch.get() && speed < 0 || !bottomlimitSwitch.get() && speed > 0) {
      lock();
      return;
    }

    left.set(speed);
    right.set(speed);
  }

  public void stop() {
    left.set(0);
    right.set(0);
  }

  public void addSpeed() {
    double spd_left = left.get();
    double spd_right = right.get();

    if (spd_left >= 1 || spd_right >= 1) {
      return;
    }

    left.set(spd_left + DEFAULT_SPEED);
    right.set(spd_right + DEFAULT_SPEED);
  }

  public void subtractSpeed() {
    double spd_left = left.get();
    double spd_right = right.get();
    if (spd_left <= -1 || spd_right <= -1) {
      return;
    }

    left.set(spd_left - DEFAULT_SPEED);
    right.set(spd_right - DEFAULT_SPEED);
  }
}