// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX RightShooterMotor;
  private final Slot0Configs Slot0Config;
  private final VelocityVoltage ShooterRequest;
  private final MotionMagicVoltage RotateMotionMagic;
  private final Slot0Configs Slot0ConfigRotate;
  private final TalonFX LeftShooterMotor;
  //private final SimpleMotorFeedforward RotateFeedForward;

  private final TalonFXS RotateMotor;
  private final TalonFXS HoodMotor;

  private final CANdi ShooterCANdi;
  public double debugTargetRPM; //This RPM target can be changed with buttons. Used for getting RPMs for distance shooting
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    debugTargetRPM = 0;
    ShooterCANdi = new CANdi(ShooterConstants.kShooterCANdi);
    CANdiConfiguration CANdiConfig = new CANdiConfiguration();
    CANdiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
    CANdiConfig.PWM1.AbsoluteSensorOffset = ShooterConstants.kRotateMotorOffset;
    ShooterCANdi.getConfigurator().apply(CANdiConfig);
    //Setup right shooter motor
    RightShooterMotor = new TalonFX(ShooterConstants.kRightShooterMotor);
    TalonFXConfiguration RightShooterConfig = new TalonFXConfiguration(); //TODO: Make an external helper function for this stuff?
    RightShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    RightShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    RightShooterMotor.getConfigurator().apply(RightShooterConfig);

    //PID for Right shooter
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
    RotateConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    RotateConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    RotateConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANdiPWM1;
    RotateConfig.ExternalFeedback.FeedbackRemoteSensorID = ShooterConstants.kShooterCANdi;
    //RotateConfig.ExternalFeedback.SensorToMechanismRatio = 10;
    //RotateConfig.ExternalFeedback.RotorToSensorRatio = mmaybe we should fuse? more work on formulas
    RotateConfig.MotionMagic.MotionMagicCruiseVelocity = 2;
    RotateConfig.MotionMagic.MotionMagicAcceleration = 4;
    RotateConfig.MotionMagic.MotionMagicJerk = 40;
    RotateMotor.getConfigurator().apply(RotateConfig);

    //RotateFeedForward = new SimpleMotorFeedforward(0, 0);
    //PID for Rotate
    
    Slot0ConfigRotate = new Slot0Configs();
    Slot0ConfigRotate.kS = 0.0; // Add 0.1 V output to overcome static friction
    Slot0ConfigRotate.kV = 0; // A velocity target of 1 rps results in 0.12 V output //Feedforward
    Slot0ConfigRotate.kP = 10; // An error of 1 rps results in 0.11 V output
    Slot0ConfigRotate.kI = 0.0; // no output for integrated error
    Slot0ConfigRotate.kD = 0; // no output for error derivative

    RotateMotor.getConfigurator().apply(Slot0ConfigRotate);
    RotateMotionMagic = new MotionMagicVoltage(0).withSlot(0);

    //Setup the hood motor
    HoodMotor = new TalonFXS(ShooterConstants.kHoodMotor);
    TalonFXSConfiguration HoodConfig = new TalonFXSConfiguration(); //TODO: Make an external helper function for this stuff?
    HoodConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    HoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    HoodMotor.getConfigurator().apply(HoodConfig);
  }
  /**
   * Rotates the shooter. This must be called periodically for safety checks to work!
   * 
   * @param speed The power to rotate the shooter with.
   */
  public void setRotate(double speed){
    if((speed >= 0 && getShooterDegrees() < ShooterConstants.kRotatateMax) //Safety Cehcks
    || speed <= 0 && getShooterDegrees() > ShooterConstants.kRotatateMin){
      RotateMotor.set(speed);
    }else{
      RotateMotor.set(0);
      System.out.println("WARNING: Trying to rotate turret out of safe zone!");
    }
  }
  public void setRotatePosition(double degrees){
    double rawPoint = getShooterRawFromDegrees(degrees);
    if (degrees > ShooterConstants.kRotatateMin && degrees < ShooterConstants.kRotatateMax){
      RotateMotor.setControl(RotateMotionMagic.withPosition(rawPoint));
    }else{
      System.out.println("WARNING: Trying to set PID past turret safe limits!");
    }
    
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
    RightShooterMotor.set(speed);
  }
  /**
   * Set the shooter to a desired RPM using a PID & Feedforward
   * 
   * @param RPM The RPM to set the shooter to.
   */
  public void setShooterRPM(double RPM){
    RightShooterMotor.setControl(ShooterRequest.withVelocity(RPM/60));
  }
  /**
   * Return true if the shooter is within 100 RPM of the target
   * @param targetRPM
   * @return True if we are near target
   */
  public boolean isShooterAtRPM(double targetRPM){
    double shooterRPM = RightShooterMotor.getVelocity().getValueAsDouble()*60;
    double shooterError = Math.abs(shooterRPM - targetRPM);
    return shooterError < 100; //We are at target RPM if the shooter is off by less than 100 RPM
  }
  public void incrementDebugRPM(double increment){
    debugTargetRPM = debugTargetRPM + increment;
  }
  public double getShooterRAWAngle(){
    return ShooterCANdi.getPWM1Position().getValueAsDouble();
  }
  public double getShooterDegrees(){
    return (getShooterRAWAngle() * 360/10) + 180;
  }
  public double getShooterRawFromDegrees(double degrees){
    return (degrees - 180) / (360.0 / 10.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", RightShooterMotor.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Debug Target RPM", debugTargetRPM);
    SmartDashboard.putNumber("Shooter RAW Angle: ", getShooterRAWAngle());
    SmartDashboard.putNumber("Shooter Degrees: ", getShooterDegrees());
  }
}
