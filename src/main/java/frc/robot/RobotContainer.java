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
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmUp;
import frc.robot.commands.DefaultShooter;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.ShooterFeedCustomRPM;
import frc.robot.commands.ShooterFullAuto;
import frc.robot.commands.ShooterFullDistance;
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
    private final ShooterFullAuto m_ShooterAutoHub = new ShooterFullAuto(m_ShooterSubsystem, m_FeederSubsystem, m_AgitatorSubsystem,() -> drivetrain.getState().Pose, FieldConstants.kHubPosition, 0.99);
    private final ShooterRevAuto m_ShooterRevHub = new ShooterRevAuto(m_ShooterSubsystem, () -> drivetrain.getState().Pose, FieldConstants.kHubPosition);
    private final IntakeRun m_IntakeRun = new IntakeRun(m_IntakeSubsystem, 0.80);
    private final ArmUp m_ArmUp = new ArmUp(m_WinchSubsystem, .15);
    private final ArmDown m_ArmDown = new ArmDown(m_WinchSubsystem, .15);
    //Drive Conffig
    private double MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double FullMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double SlowSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentric fullDrive = new SwerveRequest.FieldCentric()
            .withDeadband(FullMaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric slowDrive = new SwerveRequest.FieldCentric()
            .withDeadband(SlowSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

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
        NamedCommands.registerCommand("ArmDown", m_ArmDown);
    }

    private void configureBindings() {
        //m_ShooterSubsystem.setDefaultCommand(new DefaultShooter(m_ShooterSubsystem, () -> driver_joystick.getRightTriggerAxis()));
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
        //dont delete stuff below, used to get ranges
        //driver_joystick.leftBumper().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.incrementDebugRPM(100), () -> m_ShooterSubsystem.setShooter(0), m_ShooterSubsystem));
        //driver_joystick.rightBumper().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.incrementDebugRPM(-100), () -> m_ShooterSubsystem.setShooter(0), m_ShooterSubsystem));
        //driver_joystick.a().whileTrue(new ShooterFeedCustomRPM(m_ShooterSubsystem, m_FeederSubsystem, m_AgitatorSubsystem, 0.99));

        //Points robot at specified spot
        /*driver_joystick.x().whileTrue(drivetrain.applyRequest(() -> driveAngle.withVelocityX(-driver_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(
                        Utils.getAngleBetweenPointsForShooter(drivetrain.getState().Pose.getTranslation(), Constants.FieldConstants.Red.kGoalPosition)
                    )));*/
        //Max speed drive button
        driver_joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
                fullDrive.withVelocityX(-driver_joystick.getLeftY() * FullMaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_joystick.getLeftX() * FullMaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

        driver_joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
                slowDrive.withVelocityX(-driver_joystick.getLeftY() * SlowSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_joystick.getLeftX() * SlowSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

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

        //TEMP testing stuff
        operator_joystick.start().whileTrue(m_ArmUp);
        operator_joystick.back().whileTrue(m_ArmDown);

        //Intake Controls
        operator_joystick.leftBumper().whileTrue(m_IntakeRun);

        //Operator Controls
        //Stuck Ball
        operator_joystick.a().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setShooter(-0.3), () -> m_ShooterSubsystem.setShooter(0), m_ShooterSubsystem));
        operator_joystick.a().whileTrue(Commands.startEnd(() -> m_FeederSubsystem.setFeeder(-0.3), () -> m_FeederSubsystem.setFeeder(0), m_FeederSubsystem));
        operator_joystick.a().whileTrue(Commands.startEnd(() -> m_AgitatorSubsystem.setAgitator(-0.30), () -> m_AgitatorSubsystem.setAgitator(0), m_AgitatorSubsystem));
        //Shoot Balls
        operator_joystick.rightBumper().whileTrue(m_ShooterAutoHub);
        operator_joystick.rightTrigger().whileTrue(new ShooterFullDistance(m_ShooterSubsystem, m_FeederSubsystem, m_AgitatorSubsystem,() -> drivetrain.getState().Pose, FieldConstants.kHubPosition, 0.99));
        //Pass Balls Left
        operator_joystick.x().whileTrue(new ShooterFullAuto(m_ShooterSubsystem, m_FeederSubsystem, m_AgitatorSubsystem, () -> drivetrain.getState().Pose, FieldConstants.kPassTestSpotLeft, .75));
        //Pass Balls Right
        operator_joystick.b().whileTrue(new ShooterFullAuto(m_ShooterSubsystem, m_FeederSubsystem, m_AgitatorSubsystem,() -> drivetrain.getState().Pose, FieldConstants.kPassTestSpotRight, .75));
        //Manual Turret Controls
        //Hood Angle
        operator_joystick.povUp().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setHood(.05), () -> m_ShooterSubsystem.setHood(0), m_ShooterSubsystem));
        operator_joystick.povDown().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setHood(-.05), () -> m_ShooterSubsystem.setHood(0), m_ShooterSubsystem));
        //Turret Angle
        operator_joystick.povLeft().whileTrue(new ShooterManualRotate(m_ShooterSubsystem, 0.05));
        operator_joystick.povRight().whileTrue(new ShooterManualRotate(m_ShooterSubsystem, -0.05));

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
    }
}
