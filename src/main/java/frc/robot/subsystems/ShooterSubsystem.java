// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX RightShooterMotor;
  private final Slot0Configs Slot0Config;
  private final VelocityVoltage ShooterRequest;
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

    //PID for Right shooter //TODO: Tune!
    Slot0Config = new Slot0Configs();
    Slot0Config.kS = 0.0; // Add 0.1 V output to overcome static friction
    Slot0Config.kV = 0.115; // A velocity target of 1 rps results in 0.12 V output //Feedforward
    Slot0Config.kP = 0.3
    ; // An error of 1 rps results in 0.11 V output
    Slot0Config.kI = 0; // no output for integrated error
    Slot0Config.kD = 0; // no output for error derivative
    RightShooterMotor.getConfigurator().apply(Slot0Config);
    ShooterRequest = new VelocityVoltage(0).withSlot(0);

    //Setup left shooter motor (follows right)
    LeftShooterMotor = new TalonFX(ShooterConstants.kLeftShooterMotor);
    TalonFXConfiguration LeftShooterConfig = new TalonFXConfiguration(); //TODO: Make an external helper function for this stuff?
    LeftShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    LeftShooterMotor.setControl(new Follower(RightShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
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
    //LeftShooterMotor.set(speed);
  }
  /**
   * Set the shooter to a desired RPM using a PID & Feedforward
   * 
   * @param RPM The RPM to set the shooter to.
   */
  public void setShooterRPM(double RPM){
    RightShooterMotor.setControl(ShooterRequest.withVelocity(RPM/60));
  }
  public boolean isShooterAtRPM(double targetRPM){
    double shooterRPM = RightShooterMotor.getVelocity().getValueAsDouble()*60;
    double shooterError = Math.abs(shooterRPM - targetRPM);
    return shooterError < 100; //We are at target RPM if the shooter is off by less than 100 RPM
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", RightShooterMotor.getVelocity().getValueAsDouble()*60);
  }
}
