package ravenrobotics.robot2026;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.Constants.FlywheelConstants;
import ravenrobotics.robot2026.commands.PIDToPoseCommand;
import ravenrobotics.robot2026.subsystems.*;
import ravenrobotics.robot2026.subsystems.FeederSubsystem.FeederDirection;
import ravenrobotics.robot2026.subsystems.IntakeSubsystem.IntakeDirection;
import ravenrobotics.robot2026.subsystems.PivotSubsystem.PivotPosition;

/**
 * Subsystem for high-level control of the robot. Heavily inspired from Team 360's SuperStructure subsystem in their 2026 robot code.
 */
public class Superstructure extends SubsystemBase {
    
    private final CommandSwerveDrivetrain driveSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final PivotSubsystem pivotSubsystem;

    private SuperstructureState previousState = SuperstructureState.STOP;
    private SuperstructureState currentState = SuperstructureState.STOP;

    private PhoenixPIDController shootRotationController = new PhoenixPIDController(5.0, 0, 0);

    public enum SuperstructureState {
        STOP,
        IDLE,
        INTAKE,
        IDLE_INTAKE_OUT,
        OUTTAKE,
        SHOOT,
        PASS,
    }

    public Superstructure(
        CommandSwerveDrivetrain drivetrain,
        FeederSubsystem feeder,
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        IntakeSubsystem intake,
        PivotSubsystem pivot) 
    {
        this.driveSubsystem = drivetrain;
        this.feederSubsystem = feeder;
        this.flywheelSubsystem = flywheel;
        this.hoodSubsystem = hood;
        this.intakeSubsystem = intake;
        this.pivotSubsystem = pivot;
    }

    public void setState(SuperstructureState newState) {
        previousState = currentState;

        currentState = newState;
    }

    public Command setStateCommand(SuperstructureState newState) {
        return this.runOnce(() -> setState(newState));
    }

    private void updateState() {
        switch (currentState) {
            case STOP -> stopState();
            case IDLE -> idleState();
            case INTAKE -> intakeState();
            case IDLE_INTAKE_OUT -> idleIntakeState();
            case OUTTAKE -> outtakeState();
            case SHOOT -> shootState();
        }
    }

    private void stopState() {
        flywheelSubsystem.stopFlywheel();
        flywheelSubsystem.stopColumn();

        feederSubsystem.setFeeder(FeederDirection.FEEDER_STOP);
        intakeSubsystem.setIntakeDirection(IntakeDirection.INTAKE_STOP);

        hoodSubsystem.stopActuators();

        pivotSubsystem.setPivot(PivotPosition.PIVOT_STOP);
    }

    private void idleState() {
        flywheelSubsystem.idleFlywheel();
        flywheelSubsystem.stopColumn();

        feederSubsystem.setFeeder(FeederDirection.FEEDER_STOP);
        intakeSubsystem.setIntakeDirection(IntakeDirection.INTAKE_STOP);

        pivotSubsystem.setPivot(PivotPosition.PIVOT_IN);
    }

    private void intakeState() {
        pivotSubsystem.setPivot(PivotPosition.PIVOT_OUT);

        intakeSubsystem.setIntakeDirection(IntakeDirection.INTAKE_IN);
        feederSubsystem.setFeeder(FeederDirection.FEEDER_IN);

        flywheelSubsystem.runColumn(true);
    }

    private void idleIntakeState() {
        flywheelSubsystem.idleFlywheel();
        intakeSubsystem.setIntakeDirection(IntakeDirection.INTAKE_STOP);
        feederSubsystem.setFeeder(FeederDirection.FEEDER_STOP);

        pivotSubsystem.setPivot(PivotPosition.PIVOT_OUT);
    }

    private void outtakeState() {
        intakeSubsystem.setIntakeDirection(IntakeDirection.INTAKE_OUT);
        feederSubsystem.setFeeder(FeederDirection.FEEDER_OUT);

        pivotSubsystem.setPivot(PivotPosition.PIVOT_OUT);

        flywheelSubsystem.runColumn(true);
    }

    private void shootState() {
        FieldSide currFieldSide = getCurrentSide();

        Pose2d currentRobotState = driveSubsystem.getState().Pose;

        Rotation2d targetAngle;

        if (currFieldSide == FieldSide.RED) {
            targetAngle = new Rotation2d(
                FlywheelConstants.RED_HUB_POSE2D.getX() - currentRobotState.getX(),
                FlywheelConstants.RED_HUB_POSE2D.getY() - currentRobotState.getY());
        } else if (currFieldSide == FieldSide.BLUE) {
            targetAngle = new Rotation2d(
                FlywheelConstants.BLUE_HUB_POSE2D.getX() - currentRobotState.getX(),
                FlywheelConstants.BLUE_HUB_POSE2D.getY() - currentRobotState.getY());
        } else {
            return;
        }

        boolean isAtRotation = false;

        while (!isAtRotation) {
            double timestamp = Timer.getTimestamp();

            double newRotation = shootRotationController.calculate(
                driveSubsystem.getState().Pose.getRotation().getRadians(),
                targetAngle.getRadians(),
                timestamp);

            System.out.println("Cmd: " + newRotation);

            driveSubsystem.setControl(new SwerveRequest.RobotCentric().withRotationalRate(newRotation));

            if (shootRotationController.atSetpoint()) isAtRotation = true;
        }

        driveSubsystem.setControl(new SwerveRequest.RobotCentric());

        Matrix<N2, N1> shotParams = FlywheelConstants.SHOT_TREE.get(getDistanceToHub());

        hoodSubsystem.setPosition(shotParams.get(1, 0));

        flywheelSubsystem.runFlywheel(shotParams.get(0, 0));

        System.out.println(shotParams.get(1, 0));

        while (!hoodSubsystem.atSetpoint()) {
            hoodSubsystem.setPosition(shotParams.get(1, 0));
        }

        if (flywheelSubsystem.atSetpoint()) {
            feederSubsystem.setFeeder(FeederDirection.FEEDER_IN);
            flywheelSubsystem.runColumn(false);
        } else {
            feederSubsystem.setFeeder(FeederDirection.FEEDER_STOP);
            flywheelSubsystem.stopColumn();
        }
    }

    private double getDistanceToHub() {
        double distanceToLocalHub = -1.0;

        if (getCurrentSide() == FieldSide.BLUE) {
            distanceToLocalHub = driveSubsystem.getState().Pose.getTranslation().getDistance(FlywheelConstants.BLUE_HUB_POSE2D);
        } else if (getCurrentSide() == FieldSide.RED) {
            distanceToLocalHub = driveSubsystem.getState().Pose.getTranslation().getDistance(FlywheelConstants.RED_HUB_POSE2D);
        }

        return distanceToLocalHub;
    }

    private enum FieldSide {
        BLUE,
        RED,
        CENTER
    }

    private FieldSide getCurrentSide() {
        if (driveSubsystem.getState().Pose.getX() > 11) {
            return FieldSide.RED;
        } else if (driveSubsystem.getState().Pose.getX() < 6) {
            return FieldSide.BLUE;
        } else {
            return FieldSide.CENTER;
        }
    }

    @Override
    public void periodic() {
        DogLog.log("Superstructure/CurrentState", currentState);
        DogLog.log("Superstructure/PreviousState", previousState);

        DogLog.log("Superstructure/DistanceToHub", getDistanceToHub(), Meters);

        updateState();
    }
}
