// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot2026;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import ravenrobotics.robot2026.Superstructure.SuperstructureState;
import ravenrobotics.robot2026.generated.TunerConstants;
import ravenrobotics.robot2026.subsystems.CommandSwerveDrivetrain;
import ravenrobotics.robot2026.subsystems.PivotSubsystem;
import ravenrobotics.robot2026.subsystems.VisionSubsystem;
import ravenrobotics.robot2026.subsystems.IntakeSubsystem.IntakeDirection;
import ravenrobotics.robot2026.subsystems.PivotSubsystem.PivotPosition;
import ravenrobotics.robot2026.subsystems.IntakeSubsystem;
import ravenrobotics.robot2026.subsystems.FeederSubsystem;
import ravenrobotics.robot2026.subsystems.FlywheelSubsystem;
import ravenrobotics.robot2026.subsystems.HoodSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final DriveTelemetry logger = new DriveTelemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem(); 
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    @SuppressWarnings("unused")
    private final VisionSubsystem VisionSubsystem = new VisionSubsystem(drivetrain::addVisionMeasurement);

    private final Superstructure superStructure = new Superstructure(
        drivetrain,
        feederSubsystem,
        flywheelSubsystem,
        hoodSubsystem,
        intakeSubsystem,
        pivotSubsystem);

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        drivetrain.configurePathPlanner();

        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException("AutoBuilder failed to configure after configurePathPlanner()");
        }

        NamedCommands.registerCommand("deployIntake", pivotSubsystem.setPivotCommand(PivotPosition.PIVOT_OUT));
        NamedCommands.registerCommand("ssIdle", superStructure.setStateCommand(SuperstructureState.IDLE));
        NamedCommands.registerCommand("ssIdleIntakeOut", superStructure.setStateCommand(SuperstructureState.IDLE_INTAKE_OUT));
        NamedCommands.registerCommand("ssShoot", superStructure.setStateCommand(SuperstructureState.SHOOT));

        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
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

        //SysID stuff
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftTrigger().whileTrue(superStructure.setStateCommand(SuperstructureState.INTAKE)).onFalse(superStructure.setStateCommand(SuperstructureState.IDLE_INTAKE_OUT));

        joystick.y().onTrue(superStructure.setStateCommand(SuperstructureState.IDLE));

        // joystick.povUp().whileTrue(new InstantCommand(() -> hoodSubsystem.runActuators(false))).onFalse(new InstantCommand(() -> hoodSubsystem.stopActuators()));
        // joystick.povDown().whileTrue(new InstantCommand(() -> hoodSubsystem.runActuators(true))).onFalse(new InstantCommand(() -> hoodSubsystem.stopActuators()));

        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.leftBumper().onTrue(superStructure.setStateCommand(SuperstructureState.UNJAM)).onFalse(superStructure.setStateCommand(SuperstructureState.IDLE_INTAKE_OUT));

        joystick.rightTrigger().whileTrue(superStructure.setStateCommand(SuperstructureState.SHOOT)).onFalse(superStructure.setStateCommand(SuperstructureState.IDLE_INTAKE_OUT));

        // Set the brake
        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.setControl(brake)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // public void resetSS() {
    //     superStructure.setState(SuperstructureState.IDLE);
    // }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
