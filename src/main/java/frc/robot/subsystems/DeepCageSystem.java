package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeepCageSystem extends SubsystemBase {

  private final TalonFX motor1;
  private final TalonFX motor2;

  public DeepCageSystem() {
    motor1 = new TalonFX(Constants.DeepCage.M_ID_1);
    motor2 = new TalonFX(Constants.DeepCage.M_ID_2);
  }

  public void setSpeed(double speed) {
    motor1.set(speed * Constants.DeepCage.GEARBOX_RATIO);
    motor2.set(speed * Constants.DeepCage.GEARBOX_RATIO);
  }
}
