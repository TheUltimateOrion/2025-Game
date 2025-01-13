// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_1;
  private final TalonFX m_2;

  /** Creates a new IntakeSubsystem. */
  @SuppressWarnings("removal")
  public IntakeSubsystem(int m_1ID, boolean m_1Inverted, int m_2ID, boolean m_2Inverted) {

    m_1 = new TalonFX(m_1ID, "*");
    m_2 = new TalonFX(m_2ID, "*");

    m_1.setInverted(m_1Inverted);
    m_2.setInverted(m_2Inverted);
    
    // MotorOutputConfigs config1 = new MotorOutputConfigs();
    // MotorOutputConfigs config2 = new MotorOutputConfigs();
    // config1.Inverted = m_1Inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    // config1.Inverted = m_2Inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    // m_1.getConfigurator().apply(config1);
    // m_2.getConfigurator().apply(config2);

  }

  public void setMotors(double speed){
    m_1.set(speed);
    m_2.set(speed);
  }

  public boolean isRunning(){
    if (Math.abs(m_1.getVelocity().getValueAsDouble()) > 0){
      return true;
    } else{
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isRunning()){
      SmartDashboard.putString("Intake Status", "Running");
    } else{
      SmartDashboard.putString("Intake Status", "Idle");
    }
  }
}
