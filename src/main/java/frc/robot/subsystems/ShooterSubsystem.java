// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX RightShooterMotor;
  private final TalonFX LeftShooterMotor;

  private final TalonFXS RotateMotor;
  private final TalonFXS HoodMotor;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    //Setup right shooter motor
    RightShooterMotor = new TalonFX(ShooterConstants.kRightShooterMotor);
    TalonFXConfiguration RightShooterConfig = new TalonFXConfiguration(); //TODO: Make an external helper function for this stuff?
    RightShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    RightShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    RightShooterMotor.getConfigurator().apply(RightShooterConfig);
    //Setup left shooter motor
    LeftShooterMotor = new TalonFX(ShooterConstants.kLeftShooterMotor);
    TalonFXConfiguration LeftShooterConfig = new TalonFXConfiguration(); //TODO: Make an external helper function for this stuff?
    LeftShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    LeftShooterMotor.getConfigurator().apply(LeftShooterConfig);

    //Setup the rotate motor
    RotateMotor = new TalonFXS(ShooterConstants.kRotateMotor);
    TalonFXSConfiguration RotateConfig = new TalonFXSConfiguration(); //TODO: Make an external helper function for this stuff?
    RotateConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    RotateConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    RotateMotor.getConfigurator().apply(RotateConfig);

    //Setup the hood motor
    HoodMotor = new TalonFXS(ShooterConstants.kHoodMotor);
    TalonFXSConfiguration HoodConfig = new TalonFXSConfiguration(); //TODO: Make an external helper function for this stuff?
    HoodConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    HoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    HoodMotor.getConfigurator().apply(HoodConfig);
  }
  /**
   * Rotates the shooter
   * 
   * @param speed The power to rotate the shooter with.
   */
  public void setRotate(double speed){
    RotateMotor.set(speed);
  }
  /**
   * Rotates the hood
   * 
   * @param speed The power to rotate the hood with. Postitive is hood extend out. Negative is down
   */
  public void setHood(double speed){
    HoodMotor.set(speed);
  }
  /**
   * Set both shooter wheels
   * 
   * @param speed The power to shoot with. Positive is shoot out. Negative is back.
   */
  public void setShooter(double speed){
    //System.out.println("Shooting with: " + speed);
    RightShooterMotor.set(speed);
    LeftShooterMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
