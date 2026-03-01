// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DefaultShooter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    //Joysticks
    private final CommandXboxController joystick = new CommandXboxController(0); //TODO: Add constants, rename
    //Subsystems
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    //Drive Conffig
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        m_ShooterSubsystem.setDefaultCommand(new DefaultShooter(m_ShooterSubsystem, () -> joystick.getRightTriggerAxis()));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/
        //Turret Controls
        joystick.povUp().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setHood(.05), () -> m_ShooterSubsystem.setHood(0), m_ShooterSubsystem));
        joystick.povDown().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setHood(-.05), () -> m_ShooterSubsystem.setHood(0), m_ShooterSubsystem));
        joystick.povLeft().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRotate(-.05), () -> m_ShooterSubsystem.setRotate(0), m_ShooterSubsystem));
        joystick.povRight().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRotate(.05), () -> m_ShooterSubsystem.setRotate(0), m_ShooterSubsystem));

        //Intake Controls
        joystick.y().whileTrue(Commands.startEnd(() -> m_IntakeSubsystem.setIntake(.45), () -> m_IntakeSubsystem.setIntake(0), m_IntakeSubsystem));
        joystick.x().whileTrue(Commands.startEnd(() -> m_IntakeSubsystem.setFeeder(.50), () -> m_IntakeSubsystem.setFeeder(0), m_IntakeSubsystem));
        joystick.b().whileTrue(Commands.startEnd(() -> m_IntakeSubsystem.setFeeder(-.20), () -> m_IntakeSubsystem.setFeeder(0), m_IntakeSubsystem));

        joystick.rightBumper().whileTrue(Commands.startEnd(() -> m_IntakeSubsystem.setArm(.10), () -> m_IntakeSubsystem.setArm(0), m_IntakeSubsystem));
        joystick.leftBumper().whileTrue(Commands.startEnd(() -> m_IntakeSubsystem.setArm(-.05), () -> m_IntakeSubsystem.setArm(0), m_IntakeSubsystem));

        //Shooter Controls
        joystick.leftTrigger().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setShooter(-0.30), () -> m_ShooterSubsystem.setShooter(0), m_ShooterSubsystem));
        //joystick.rightTrigger().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setShooter(joystick.getRightTriggerAxis()), () -> m_ShooterSubsystem.setShooter(0), m_ShooterSubsystem));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
