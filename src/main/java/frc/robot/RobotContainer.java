// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DefaultShooter;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.ShooterFeed;
import frc.robot.commands.ShooterFeedAutoRPM;
import frc.robot.commands.ShooterFeedCustomRPM;
import frc.robot.commands.ShooterFullAuto;
import frc.robot.commands.ShooterManualRotate;
import frc.robot.commands.ShooterRevAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WinchSubsystem;

public class RobotContainer {
    //Joysticks
    private final CommandXboxController driver_joystick = new CommandXboxController(0);
    private final CommandXboxController operator_joystick = new CommandXboxController(1);
    //Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
    private final AgitatorSubsystem m_AgitatorSubsystem = new AgitatorSubsystem();
    private final WinchSubsystem m_WinchSubsystem = new WinchSubsystem();
    //Reused Commands
    private final ShooterFullAuto m_ShooterAutoHub = new ShooterFullAuto(m_ShooterSubsystem, m_FeederSubsystem, () -> drivetrain.getState().Pose, FieldConstants.kHubPosition, .6);
    private final ShooterRevAuto m_ShooterRevHub = new ShooterRevAuto(m_ShooterSubsystem, () -> drivetrain.getState().Pose, FieldConstants.kHubPosition);
    private final IntakeRun m_IntakeRun = new IntakeRun(m_IntakeSubsystem, 0.50);
    //Drive Conffig
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //For aiming at a pont
    private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withHeadingPID(5, 0, 0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            

    private final Telemetry logger = new Telemetry(MaxSpeed);

    

    

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        registerCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);
    }

    public void registerCommands(){
        NamedCommands.registerCommand("ShooterAutoHub", m_ShooterAutoHub);
        NamedCommands.registerCommand("IntakeRun", m_IntakeRun);
        NamedCommands.registerCommand("ShooterRevHub", m_ShooterRevHub);
        NamedCommands.registerCommand("AgitatorForward", Commands.startEnd(() -> m_AgitatorSubsystem.setAgitator(0.3), () -> m_AgitatorSubsystem.setAgitator(0), m_AgitatorSubsystem));
        NamedCommands.registerCommand("WinchOut", Commands.startEnd(() -> m_WinchSubsystem.setWinch(-0.5), () -> m_WinchSubsystem.setWinch(0), m_WinchSubsystem).withTimeout(1));
    }

    private void configureBindings() {
        m_ShooterSubsystem.setDefaultCommand(new DefaultShooter(m_ShooterSubsystem, () -> driver_joystick.getRightTriggerAxis()));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        //Points robot at specified spot
        /*driver_joystick.x().whileTrue(drivetrain.applyRequest(() -> driveAngle.withVelocityX(-driver_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(
                        Utils.getAngleBetweenPointsForShooter(drivetrain.getState().Pose.getTranslation(), Constants.FieldConstants.Red.kGoalPosition)
                    )));*/


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*driver_joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver_joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver_joystick.getLeftY(), -driver_joystick.getLeftX()))
        ));*/
        //Turret Controls
        driver_joystick.povUp().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setHood(.05), () -> m_ShooterSubsystem.setHood(0), m_ShooterSubsystem));
        driver_joystick.povDown().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setHood(-.05), () -> m_ShooterSubsystem.setHood(0), m_ShooterSubsystem));
        driver_joystick.povLeft().whileTrue(new ShooterManualRotate(m_ShooterSubsystem, 0.05));
        driver_joystick.povRight().whileTrue(new ShooterManualRotate(m_ShooterSubsystem, -0.05));

        //Intake Controls
        operator_joystick.y().toggleOnTrue(m_IntakeRun);
        //driver_joystick.x().whileTrue(Commands.startEnd(() -> m_FeederSubsystem.setFeeder(.50), () -> m_FeederSubsystem.setFeeder(0), m_IntakeSubsystem));
        //driver_joystick.b().whileTrue(Commands.startEnd(() -> m_FeederSubsystem.setFeeder(-.20), () -> m_FeederSubsystem.setFeeder(0), m_IntakeSubsystem));

        //driver_joystick.rightBumper().whileTrue(Commands.startEnd(() -> m_IntakeSubsystem.setArm(.10), () -> m_IntakeSubsystem.setArm(0), m_IntakeSubsystem));
        //driver_joystick.leftBumper().whileTrue(Commands.startEnd(() -> m_IntakeSubsystem.setArm(-.05), () -> m_IntakeSubsystem.setArm(0), m_IntakeSubsystem));
        //below are for debug
        //operator_joystick.leftBumper().toggleOnTrue(Commands.startEnd(() -> m_AgitatorSubsystem.setAgitator(0.5), () -> m_AgitatorSubsystem.setAgitator(0), m_AgitatorSubsystem));
        operator_joystick.leftTrigger().whileTrue(Commands.startEnd(() -> m_AgitatorSubsystem.setAgitator(-0.5), () -> m_AgitatorSubsystem.setAgitator(0), m_AgitatorSubsystem)); //Check to be sure this works when the shooter is active. Todo, change it so this is left trigger with the feeder.
        //driver_joystick.rightBumper().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.incrementDebugRPM(200), () -> m_ShooterSubsystem.incrementDebugRPM(0), m_ShooterSubsystem));
        //driver_joystick.leftBumper().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRotatePosition(100), () -> m_ShooterSubsystem.setRotate(0), m_ShooterSubsystem));
        //driver_joystick.rightBumper().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRotatePosition(170), () -> m_ShooterSubsystem.setRotate(0), m_ShooterSubsystem));

        //Shooter Controls
        driver_joystick.leftTrigger().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setShooter(-0.30), () -> m_ShooterSubsystem.setShooter(0), m_ShooterSubsystem));
        driver_joystick.leftTrigger().whileTrue(Commands.startEnd(() -> m_FeederSubsystem.setFeeder(-0.30), () -> m_FeederSubsystem.setFeeder(0), m_FeederSubsystem));
        driver_joystick.start().toggleOnTrue(new ShooterFeedCustomRPM(m_ShooterSubsystem, m_FeederSubsystem, .50));
        //joystick.b().toggleOnTrue(new ShooterFeedAutoRPM(m_ShooterSubsystem, m_FeederSubsystem, () -> drivetrain.getDistanceFromTarget(FieldConstants.Red.kGoalPosition),.50));
        operator_joystick.leftTrigger().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setShooter(-0.30), () -> m_ShooterSubsystem.setShooter(0), m_ShooterSubsystem));
        operator_joystick.leftTrigger().whileTrue(Commands.startEnd(() -> m_FeederSubsystem.setFeeder(-0.30), () -> m_FeederSubsystem.setFeeder(0), m_FeederSubsystem));
        operator_joystick.a().toggleOnTrue(m_ShooterAutoHub);
        operator_joystick.a().toggleOnTrue(Commands.startEnd(() -> m_AgitatorSubsystem.setAgitator(0.3), () -> m_AgitatorSubsystem.setAgitator(0), m_AgitatorSubsystem));
        operator_joystick.x().toggleOnTrue(new ShooterFullAuto(m_ShooterSubsystem, m_FeederSubsystem, () -> drivetrain.getState().Pose, FieldConstants.kPassTestSpotLeft, .75));
        operator_joystick.x().toggleOnTrue(Commands.startEnd(() -> m_AgitatorSubsystem.setAgitator(0.3), () -> m_AgitatorSubsystem.setAgitator(0), m_AgitatorSubsystem));
        operator_joystick.b().toggleOnTrue(new ShooterFullAuto(m_ShooterSubsystem, m_FeederSubsystem, () -> drivetrain.getState().Pose, FieldConstants.kPassTestSpotRight, .75));
        operator_joystick.b().toggleOnTrue(Commands.startEnd(() -> m_AgitatorSubsystem.setAgitator(0.3), () -> m_AgitatorSubsystem.setAgitator(0), m_AgitatorSubsystem));
        //driver_joystick.rightTrigger().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setShooter(driver_joystick.getRightTriggerAxis()), () -> m_ShooterSubsystem.setShooter(0), m_ShooterSubsystem));
        operator_joystick.start().whileTrue(Commands.startEnd(() -> m_WinchSubsystem.setWinch(0.3), () -> m_WinchSubsystem.setWinch(0), m_WinchSubsystem));
        operator_joystick.back().whileTrue(Commands.startEnd(() -> m_WinchSubsystem.setWinch(-0.3), () -> m_WinchSubsystem.setWinch(0), m_WinchSubsystem));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driver_joystick.back().and(driver_joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driver_joystick.back().and(driver_joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driver_joystick.start().and(driver_joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driver_joystick.start().and(driver_joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver_joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        System.out.println("AUTO RUNNING: " + autoChooser.getSelected().getName());
        return autoChooser.getSelected();
        // Simple drive forward auton
        /*
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
        ); */
    }
}
