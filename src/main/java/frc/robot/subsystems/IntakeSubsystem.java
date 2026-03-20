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
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFXS BottomIntakeMotor;
  private final TalonFXS TopIntakeMotor;

  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    //Setup Bottom Intake Motor
    BottomIntakeMotor = new TalonFXS(IntakeConstants.kBottomIntakeMotor);
    TalonFXSConfiguration BottomIntakeConfig = new TalonFXSConfiguration();
    BottomIntakeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    BottomIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //BottomIntakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    BottomIntakeMotor.getConfigurator().apply(BottomIntakeConfig);

    //Setup Top Intake Motor
    TopIntakeMotor = new TalonFXS(IntakeConstants.kTopIntakeMotor);
    TalonFXSConfiguration TopIntakeConfig = new TalonFXSConfiguration();
    TopIntakeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    TopIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    TopIntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    TopIntakeMotor.getConfigurator().apply(TopIntakeConfig);

    //Setup Right Arm Motor
    //RightArmMotor = new TalonFX(IntakeConstants.kRightArmMotor);
    //TalonFXConfiguration RightArmConfig = new TalonFXConfiguration();
    //RightArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //RightArmConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //RightArmMotor.getConfigurator().apply(RightArmConfig);

    
  }
  /**
   * Set both intake motors on the arm.
   * 
   * @param speed The power to set the motors to. Positive is intake. Negative is out.
   */
  public void setIntake(double speed){
    BottomIntakeMotor.set(speed);
    TopIntakeMotor.set(speed);
  }
  /**
   * Set both arm motors.
   * 
   * @param speed The power to set the motors to. Positive is arm up. Negative is arm down.
   */
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
