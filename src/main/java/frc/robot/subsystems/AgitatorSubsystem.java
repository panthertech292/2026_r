// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AgitatorConstants;

public class AgitatorSubsystem extends SubsystemBase {
  private final TalonFX agitatorMotor;
  /** Creates a new AgitatorSubsystem. */
  public AgitatorSubsystem() {
    //Setup Left Agitator Motor
    agitatorMotor = new TalonFX(AgitatorConstants.kAgitatorMotor);
    TalonFXConfiguration agitatorMotorConfig = new TalonFXConfiguration();
    agitatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    agitatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    agitatorMotor.getConfigurator().apply(agitatorMotorConfig);
  }

  /**
   * Set both feed motors.
   * 
   * @param speed The power to set the motors to. Positive is intake. Negative is out.
   */
  public void setAgitator(double speed){
    agitatorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
