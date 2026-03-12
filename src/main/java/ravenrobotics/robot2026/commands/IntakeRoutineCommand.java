package ravenrobotics.robot2026.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot2026.subsystems.FeederSubsystem;
import ravenrobotics.robot2026.subsystems.FlywheelSubsystem;
import ravenrobotics.robot2026.subsystems.IntakeSubsystem;
import ravenrobotics.robot2026.subsystems.PivotSubsystem;
import ravenrobotics.robot2026.subsystems.FeederSubsystem.FeederDirection;
import ravenrobotics.robot2026.subsystems.IntakeSubsystem.IntakeDirection;
import ravenrobotics.robot2026.subsystems.PivotSubsystem.PivotPosition;

public class IntakeRoutineCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;

    private final IntakeRoutineMode mode;
    private boolean isFinished = false;

    public enum IntakeRoutineMode {
        INTAKE_DEPLOY,
        INTAKE_RETRACT
    }

    public IntakeRoutineCommand(IntakeRoutineMode mode, PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, FlywheelSubsystem flywheelSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        
        this.mode = mode;

        this.addRequirements(this.pivotSubsystem, this.intakeSubsystem, this.feederSubsystem, this.flywheelSubsystem);
    }

    @Override
    public void execute() {
        switch (mode) {
            case INTAKE_DEPLOY:
                pivotSubsystem.setPivot(PivotPosition.PIVOT_OUT);
                intakeSubsystem.setIntakeDirection(IntakeDirection.INTAKE_IN);
                feederSubsystem.setFeeder(FeederDirection.FEEDER_IN);
                flywheelSubsystem.runColumn(true);
                break;
            case INTAKE_RETRACT:
                pivotSubsystem.setPivot(PivotPosition.PIVOT_IN);
                intakeSubsystem.setIntakeDirection(IntakeDirection.INTAKE_STOP);
                feederSubsystem.setFeeder(FeederDirection.FEEDER_STOP);
                if (pivotSubsystem.atSetpoint()) {
                    isFinished = true;
                }
                break;
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        flywheelSubsystem.stopColumn();
        Timer.delay(0.1);
        intakeSubsystem.setIntakeDirection(IntakeDirection.INTAKE_STOP);
        feederSubsystem.setFeeder(FeederDirection.FEEDER_STOP);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
