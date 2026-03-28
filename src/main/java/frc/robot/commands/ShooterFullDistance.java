// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterInterpolationConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterFullDistance extends Command {
  private ShooterSubsystem ShooterSub;
  private FeederSubsystem FeederSub;
  private AgitatorSubsystem AgitatorSub;
  private Supplier<Pose2d> robot;
  private Translation2d target;
  private final Translation2d statTarget; 
  private double feedSpeed;
  private double distance;
  private double targetRPM;
  //private final Translation2d offset;
  /** Creates a new ShooterFeed. */
  public ShooterFullDistance(ShooterSubsystem Shooter_Subsystem, FeederSubsystem Feeder_Subsystem, AgitatorSubsystem Agitator_Subsystem, Supplier<Pose2d> robot, Translation2d targetARG, double feedSpeed) {
    ShooterSub = Shooter_Subsystem;
    FeederSub = Feeder_Subsystem;
    AgitatorSub = Agitator_Subsystem;
    this.robot = robot;
    this.target = targetARG;
    this.statTarget = targetARG;
    this.distance = 0;
    this.feedSpeed = feedSpeed;
    targetRPM = 0;
    //offset = new Translation2d(-0.1778, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ShooterSub, FeederSub, AgitatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) { //Convert input to red if needed
      if (alliance.get() == DriverStation.Alliance.Red){
        System.out.println("SHOOTER DISTANCE AUTO: We are red alliance!");
        target = Utils.getRedTranslatonFromBlue(statTarget); //Converts blue -> red
      }else{
        System.out.println("SHOOTER DISTANCE AUTO: We are blue alliance!");
      }
    }
    else{
        System.out.println("SHOOTER DISTANCE AUTO: NO ALLIANCE FOUND!");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d turretPose = robot.get().plus(ShooterConstants.TurretPositionOffset);
   
    distance = turretPose.getTranslation().getDistance(target);
    targetRPM = ShooterInterpolationConstants.rpmMAP.get(distance);
    ShooterSub.setShooterRPM(targetRPM);
    System.out.println("SHOOTER  DISTANCE: Aiming for: X" + target.getX() + " Y: " + target.getY() + " Distance: " + distance + " RPM:" + targetRPM);
    if(ShooterSub.isShooterAtRPM(targetRPM)){
      FeederSub.setFeeder(feedSpeed);
      AgitatorSub.setAgitator(AgitatorConstants.kSpeed);
    }else{
      FeederSub.setFeeder(0);
      AgitatorSub.setAgitator(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.setShooter(0);
    FeederSub.setFeeder(0);
    ShooterSub.setRotate(0);
    AgitatorSub.setAgitator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
