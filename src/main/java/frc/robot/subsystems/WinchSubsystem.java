// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class WinchSubsystem extends SubsystemBase {
  private final TalonFX WinchMotor;
  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {
    //Setup Winch Motor
    WinchMotor = new TalonFX(IntakeConstants.kWinchMotor);
    TalonFXConfiguration LeftArmConfig = new TalonFXConfiguration();
    LeftArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    LeftArmConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    WinchMotor.getConfigurator().apply(LeftArmConfig);
  }

  public void setWinch(double speed){
    WinchMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
