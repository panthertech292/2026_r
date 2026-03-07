// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
  private final TalonFX BottomFeederMotor;
  private final TalonFX TopFeederMotor;
  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    //Setup Bottom Feeder Motor
    BottomFeederMotor = new TalonFX(FeederConstants.kBottomFeederMotor);
    TalonFXConfiguration BottomFeederConfig = new TalonFXConfiguration();
    BottomFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    BottomFeederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    BottomFeederMotor.getConfigurator().apply(BottomFeederConfig);

    //Setup Top Feeder Motor
    TopFeederMotor = new TalonFX(FeederConstants.kTopFeederMotor);
    TalonFXConfiguration TopFeederConfig = new TalonFXConfiguration();
    TopFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    TopFeederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    TopFeederMotor.getConfigurator().apply(TopFeederConfig);
  }

  /**
   * Set both feed motors.
   * 
   * @param speed The power to set the motors to. Positive is intake. Negative is out.
   */
  public void setFeeder(double speed){
    BottomFeederMotor.set(speed);
    TopFeederMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
