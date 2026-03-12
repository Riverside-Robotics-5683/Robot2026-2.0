package ravenrobotics.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.MotorConfigs;
import ravenrobotics.robot2026.Constants.FeederAndIntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;

    // Intake encoder signal.
    private final StatusSignal<AngularVelocity> intakeVelocity;

    // Intake current signals.
    private final StatusSignal<Current> intakeStatorCurrent;
    private final StatusSignal<Current> intakeSupplyCurrent;

    /**
     * The direction to command the intake.
     */
    public enum IntakeDirection {
        /**
         * Intake fuel.
         */
        INTAKE_IN,
        /**
         * Outtake fuel.
         */
        INTAKE_OUT,
        /**
         * Stop the intake.
         */
        INTAKE_STOP
    }

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(FeederAndIntakeConstants.INTAKE_MOTOR);

        intakeMotor.getConfigurator().apply(MotorConfigs.intakeConfig);

        intakeVelocity = intakeMotor.getVelocity();

        intakeStatorCurrent = intakeMotor.getStatorCurrent();
        intakeSupplyCurrent = intakeMotor.getSupplyCurrent();
    }

    public void setIntakeDirection(IntakeDirection direction) {
        switch (direction) {
            case INTAKE_IN:
                intakeMotor.set(-0.9);
                break;
            case INTAKE_OUT:
                intakeMotor.set(0.9);
                break;
            case INTAKE_STOP:
                intakeMotor.stopMotor();
                break;
        }
    }

    public Command setIntakeDirectionCommand(IntakeDirection direction) {
        return this.runOnce(() -> {
            setIntakeDirection(direction);
        });
    }
    
    @Override
    public void periodic() {
        // Intake encoder.
        DogLog.log("Intake/Velocity", intakeVelocity.getValueAsDouble(), RotationsPerSecond);
        
        // Intake currents.
        DogLog.log("Intake/StatorCurrent", intakeStatorCurrent.getValueAsDouble(), Amps);
        DogLog.log("Intake/SupplyCurrent", intakeSupplyCurrent.getValueAsDouble(), Amps);
    }
}
