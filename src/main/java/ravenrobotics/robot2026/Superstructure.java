package ravenrobotics.robot2026;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.Constants.FlywheelConstants;
import ravenrobotics.robot2026.subsystems.*;
import ravenrobotics.robot2026.subsystems.FeederSubsystem.FeederDirection;
import ravenrobotics.robot2026.subsystems.FlywheelSubsystem.FlywheelIdleState;
import ravenrobotics.robot2026.subsystems.IntakeSubsystem.IntakeDirection;
import ravenrobotics.robot2026.subsystems.PivotSubsystem.PivotPosition;
import ravenrobotics.robot2026.util.HubShiftUtil;
import ravenrobotics.robot2026.util.HubShiftUtil.ShiftInfo;

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
    private boolean pivotDirection = false;
    private int pivotCounter = 0;

    private Timer shiftTimer = new Timer();

    private double tunableRpm = 3000;
    private double tunableHood = .05;

    public enum SuperstructureState {
        STOP,
        IDLE,
        INTAKE,
        IDLE_INTAKE_OUT,
        OUTTAKE,
        SHOOT,
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

        DogLog.tunable("Superstructure/Hood", tunableHood, (newHood) -> {
            this.tunableHood = newHood;
        });

        DogLog.tunable("Superstructure/RPM", tunableRpm, (newRpm) -> {
            this.tunableRpm = newRpm;
        });

        shiftTimer.reset();
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
            case SHOOT -> handleShoot();
        }
    }

    private void handleShoot() {
        var currentAlliance = DriverStation.getAlliance();
        var currentSide = getCurrentSide();

        if (currentAlliance.isEmpty()) return;
        if (currentSide.isEmpty()) {
            passShootState();
            return;
        }

        if (currentSide.get() == currentAlliance.get()) {
            hubShootState();
        } else {
            passShootState();
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
        handleFlywheelIdle();

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
        handleFlywheelIdle();

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

    private void handleFlywheelIdle() {
        var currentSide = getCurrentSide();
        var currentAlliance = DriverStation.getAlliance();

        if (currentAlliance.isEmpty()) {
            flywheelSubsystem.idleFlywheel(FlywheelIdleState.HUB);
            return;
        }
        if (currentSide.isEmpty()) {
            flywheelSubsystem.idleFlywheel(FlywheelIdleState.PASS);
            return;
        }

        if (currentSide.get() == currentAlliance.get()) {
            flywheelSubsystem.idleFlywheel(FlywheelIdleState.HUB);
        } else {
            flywheelSubsystem.idleFlywheel(FlywheelIdleState.PASS);
        }
    }

    private void passShootState() {
        Matrix<N2, N1> shotParams = FlywheelConstants.PASS_SHOT_TREE.get(getPassDistance());

        flywheelSubsystem.runFlywheel(shotParams.get(0, 0));
        hoodSubsystem.setPosition(shotParams.get(1, 0));

        if (hoodSubsystem.atSetpoint() && flywheelSubsystem.atSetpoint()) {
            hoodSubsystem.stopActuators();

            flywheelSubsystem.runColumn(false);
            feederSubsystem.setFeeder(FeederDirection.FEEDER_IN);

            cycleShakePivot();
        } else {
            flywheelSubsystem.runColumn(true);
            feederSubsystem.setFeeder(FeederDirection.FEEDER_STOP);
        }
    }

    private void hubShootState() {
        var currFieldSide = getCurrentSide();

        if (currFieldSide.isEmpty()) return;

        Pose2d currentRobotState = driveSubsystem.getState().Pose;

        Rotation2d targetAngle;

        if (currFieldSide.get() == Alliance.Red) {
            targetAngle = new Rotation2d(
                FlywheelConstants.RED_HUB_POS.getX() - currentRobotState.getX(),
                FlywheelConstants.RED_HUB_POS.getY() - currentRobotState.getY());
        } else {
            targetAngle = new Rotation2d(
                FlywheelConstants.BLUE_HUB_POS.getX() - currentRobotState.getX(),
                FlywheelConstants.BLUE_HUB_POS.getY() - currentRobotState.getY());
        }

        boolean isAtRotation = false;

        while (!isAtRotation) {
            double timestamp = Timer.getTimestamp();

            double newRotation = shootRotationController.calculate(
                driveSubsystem.getState().Pose.getRotation().getRadians(),
                targetAngle.getRadians(),
                timestamp);

            driveSubsystem.setControl(new SwerveRequest.RobotCentric().withRotationalRate(newRotation));

            if (shootRotationController.atSetpoint()) isAtRotation = true;
        }

        driveSubsystem.setControl(new SwerveRequest.RobotCentric());

        Matrix<N2, N1> shotParams = FlywheelConstants.HUB_SHOT_TREE.get(getDistanceToHub());

        flywheelSubsystem.runFlywheel(shotParams.get(0, 0));
        hoodSubsystem.setPosition(shotParams.get(1, 0));

        if (flywheelSubsystem.atSetpoint() && hoodSubsystem.atSetpoint()) {
            hoodSubsystem.stopActuators();

            driveSubsystem.setControl(new SwerveRequest.SwerveDriveBrake());

            feederSubsystem.setFeeder(FeederDirection.FEEDER_IN);
            flywheelSubsystem.runColumn(false);

            cycleShakePivot();
        } else {
            feederSubsystem.setFeeder(FeederDirection.FEEDER_OUT);
            flywheelSubsystem.runColumn(true);

            hoodSubsystem.setPosition(shotParams.get(1, 0));
        }
    }

    private void cycleShakePivot() {
        pivotCounter++;

        if (pivotCounter > 50) {
            if (!pivotDirection) {
                pivotDirection = true;
            } else {
                pivotDirection = false;
            }

            pivotCounter = 0;
        }

        if (pivotDirection) {
            pivotSubsystem.setPivot(PivotPosition.PIVOT_SHOOT_LOW);
        } else {
            pivotSubsystem.setPivot(PivotPosition.PIVOT_SHOOT_HIGH);
        }
    }

    private double getDistanceToHub() {
        double distanceToLocalHub = -1.0;

        Translation2d driveTranslation = driveSubsystem.getState().Pose.getTranslation();
        var currentSide = getCurrentSide();

        if (currentSide.isEmpty()) return distanceToLocalHub;

        if (currentSide.get() == Alliance.Red) {
            distanceToLocalHub = driveTranslation.getDistance(FlywheelConstants.RED_HUB_POS);
        } else {
            distanceToLocalHub = driveTranslation.getDistance(FlywheelConstants.BLUE_HUB_POS);
        }

        return distanceToLocalHub;
    }

    private double getPassDistance() {
        double distanceToPass = -1.0;

        Translation2d driveTranslation = driveSubsystem.getState().Pose.getTranslation();
        var currentAlliance = DriverStation.getAlliance();

        if (currentAlliance.isEmpty()) return distanceToPass;

        if (currentAlliance.get() == Alliance.Blue) {
            if (driveTranslation.getY() < 3.05) {
                distanceToPass = driveTranslation.getDistance(FlywheelConstants.BLUE_LOW_POS);
            } else if (driveTranslation.getY() > 5.0) {
                distanceToPass = driveTranslation.getDistance(FlywheelConstants.BLUE_HIGH_POS);
            }
        } else {
            if (driveTranslation.getY() < 3.05) {
                distanceToPass = driveTranslation.getDistance(FlywheelConstants.RED_LOW_POS);
            } else if (driveTranslation.getY() > 5.0) {
                distanceToPass = driveTranslation.getDistance(FlywheelConstants.RED_HIGH_POS);
            }
        }

        return distanceToPass;
    }

    private Optional<Alliance> getCurrentSide() {
        double driveX = driveSubsystem.getState().Pose.getX();

        if (driveX > 11) {
            return Optional.of(Alliance.Red);
        } else if (driveX < 6) {
            return Optional.of(Alliance.Blue);
        } else {
            return Optional.empty();
        }
    }

    @Override
    public void periodic() {
        updateState();

        DogLog.log("MatchTime", DriverStation.getMatchTime(), Seconds);

        DogLog.log("Superstructure/CurrentState", currentState);
        DogLog.log("Superstructure/PreviousState", previousState);

        DogLog.log("Superstructure/DistanceToHub", getDistanceToHub(), Meters);
        DogLog.log("Superstructure/PassDistance", getPassDistance(), Meters);

        DogLog.log("Superstructure/ShootState", getCurrentSide().isPresent());

        ShiftInfo currentShift = HubShiftUtil.getOfficialShiftInfo();

        DogLog.log("ShiftTime", currentShift.remainingTime(), Seconds);
        DogLog.log("CurrentlyActive", currentShift.active());
    }
}
