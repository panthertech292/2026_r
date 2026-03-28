// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterFeedCustomRPM extends Command {
  private ShooterSubsystem ShooterSub;
  private FeederSubsystem FeederSub;
  private AgitatorSubsystem AgitatorSub;
  private double feedSpeed;
  /** Creates a new ShooterFeed. */
  public ShooterFeedCustomRPM(ShooterSubsystem Shooter_Subsystem, FeederSubsystem Feeder_Subsystem, AgitatorSubsystem Agitator_Subsystem, double feedSpeed) {
    ShooterSub = Shooter_Subsystem;
    FeederSub = Feeder_Subsystem;
    this.feedSpeed = feedSpeed;
    AgitatorSub = Agitator_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ShooterSub, FeederSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AgitatorSub.setAgitator(AgitatorConstants.kSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSub.setShooterRPM(ShooterSub.debugTargetRPM);
    if(ShooterSub.isShooterAtRPM(ShooterSub.debugTargetRPM)){
      FeederSub.setFeeder(feedSpeed);
    }else{
      FeederSub.setFeeder(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.setShooter(0);
    FeederSub.setFeeder(0);
    AgitatorSub.setAgitator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
