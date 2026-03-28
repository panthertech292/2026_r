// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase {
  private final TalonFXS WinchMotor;
  private final Servo WinchServo;
  private boolean WinchLocked;
  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {
    //Setup Winch Motor
    WinchMotor = new TalonFXS(WinchConstants.kWinchMotor);
    TalonFXSConfiguration WinchConfig = new TalonFXSConfiguration();
    WinchConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    WinchConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    WinchConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    WinchMotor.getConfigurator().apply(WinchConfig);

    WinchServo = new Servo(WinchConstants.kWinchServo);
    WinchLocked = true; //Assume true, so we unlock
    unlockServo();
  }

  public void setWinch(double speed){
    WinchMotor.set(speed);
  }

  public void lockServo(){
    WinchServo.set(0.25);
    this.WinchLocked = true;
  }
  public void unlockServo(){
    WinchServo.set(0);
    this.WinchLocked = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
