package ravenrobotics.robot2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot2026.subsystems.FeederSubsystem;
import ravenrobotics.robot2026.subsystems.FlywheelSubsystem;
import ravenrobotics.robot2026.subsystems.IntakeSubsystem;
import ravenrobotics.robot2026.subsystems.FeederSubsystem.FeederDirection;

public class FlywheelRoutineCommand extends Command {
    private final FlywheelSubsystem flywheelSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final FeederSubsystem feederSubsystem;

    public FlywheelRoutineCommand(FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;

        this.addRequirements(this.flywheelSubsystem, this.intakeSubsystem, this.feederSubsystem);
    }

    @Override
    public void execute() {
        flywheelSubsystem.runFlywheel(flywheelSubsystem.flywheelSpeed);

        if (flywheelSubsystem.atSetpoint()) {
            flywheelSubsystem.runColumn(false);
            feederSubsystem.setFeeder(FeederDirection.FEEDER_IN);
        } else {
            flywheelSubsystem.stopColumn();
            feederSubsystem.setFeeder(FeederDirection.FEEDER_STOP);
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        flywheelSubsystem.idleFlywheel();
        flywheelSubsystem.stopColumn();
        feederSubsystem.setFeeder(FeederDirection.FEEDER_STOP);
    }
}
