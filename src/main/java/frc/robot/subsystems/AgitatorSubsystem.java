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
  private final TalonFX LeftAgitatorMotor;
  private final TalonFX RightAgitatorMotor;
  /** Creates a new AgitatorSubsystem. */
  public AgitatorSubsystem() {
    //Setup Left Agitator Motor
    LeftAgitatorMotor = new TalonFX(AgitatorConstants.kLeftAgitatorMotor);
    TalonFXConfiguration LeftAgitatorMotorConfig = new TalonFXConfiguration();
    LeftAgitatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    LeftAgitatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    LeftAgitatorMotor.getConfigurator().apply(LeftAgitatorMotorConfig);

    //Setup Right Agitator Motor
    RightAgitatorMotor = new TalonFX(AgitatorConstants.kRightAgitatorMotor);
    TalonFXConfiguration RightAgitatorMotorConfig = new TalonFXConfiguration();
    RightAgitatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    RightAgitatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    RightAgitatorMotor.getConfigurator().apply(RightAgitatorMotorConfig);
  }

  /**
   * Set both feed motors.
   * 
   * @param speed The power to set the motors to. Positive is intake. Negative is out.
   */
  public void setAgitator(double speed){
    LeftAgitatorMotor.set(speed);
    RightAgitatorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
