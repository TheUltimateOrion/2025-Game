package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSystem extends SubsystemBase {
  private final TalonFX left;
  private final TalonFX right;

  public ElevatorSystem(int leftID, int rightID) {
    left = new TalonFX(leftID);
    right = new TalonFX(rightID);
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = 1.0 / 20.0;
    left.getConfigurator().apply(feedbackConfigs);
    right.getConfigurator().apply(feedbackConfigs);
    left.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    right.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.03; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kD = 0.005; // A velocity of 1 rps results in 0.1 V output
    slot0Configs.kG = -0.26;
    slot0Configs.kS = 0.105;
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    right.getConfigurator().apply(slot0Configs);
    left.getConfigurator().apply(slot0Configs);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = 1500;
    motionMagicConfigs.MotionMagicCruiseVelocity = 1500;
    left.getConfigurator().apply(motionMagicConfigs);
    right.getConfigurator().apply(motionMagicConfigs);
  }

  public enum Direction {
    Up,
    Down,
    Stop
  };

  public void set(double point) {
    MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);

    left.setControl(request.withPosition(-point));
    right.setControl(request.withPosition(-point));
  }

  @Override
  public void periodic() {

  }

}