package ravenrobotics.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.MotorConfigs;
import ravenrobotics.robot2026.Constants.FeederAndIntakeConstants;

/**
 * Subsystem for controlling the feeder and intake mechanisms.
 */
public class PivotSubsystem extends SubsystemBase {
    
    // Intake motors.
    private final TalonFX pivotMotor;

    private PositionVoltage motorRequest = new PositionVoltage(FeederAndIntakeConstants.PIVOT_IN).withSlot(0);

    // Pivot encoder signals.
    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;

    // Pivot current signals.
    private final StatusSignal<Current> pivotStatorCurrent;
    private final StatusSignal<Current> pivotSupplyCurrent;

    /**
     * The position to command the pivot motor to.
     */
    public enum PivotPosition {
        /**
         * Brings in the intake.
         */
        PIVOT_IN,
        /**
         * Deploys the intake.
         */
        PIVOT_OUT
    }

    /**
     * Constructs the subsystem for controlling the feeder and intake mechanisms.
     */
    public PivotSubsystem() {
        // Initialize intake motors.
        pivotMotor = new TalonFX(FeederAndIntakeConstants.FEEDER_MOTOR, new CANBus("rio"));

        // Configure intake motors.
        pivotMotor.getConfigurator().apply(MotorConfigs.pivotConfig);

        // Reset pivot motor position.
        pivotMotor.setPosition(0);

        // Get status signals for pivot.
        pivotPosition = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();

        pivotStatorCurrent = pivotMotor.getStatorCurrent();
        pivotSupplyCurrent = pivotMotor.getSupplyCurrent();

        // Set high frequency for position and velocities.
        BaseStatusSignal.setUpdateFrequencyForAll(Frequency.ofRelativeUnits(100, Hertz),
            pivotPosition,
            pivotVelocity);

        // Set higher but not as high frequency for the not as important signals.
        BaseStatusSignal.setUpdateFrequencyForAll(Frequency.ofRelativeUnits(50, Hertz),
            pivotStatorCurrent,
            pivotSupplyCurrent);

        // Optimize bus usage for the pivot and intake motors.
        ParentDevice.optimizeBusUtilizationForAll(pivotMotor);

        // Register subsystem with CommandScheduler.
        this.register();
    }

    public void setPivot(PivotPosition position) {
        switch (position) {
            case PIVOT_IN:
                pivotMotor.setControl(motorRequest.withPosition(FeederAndIntakeConstants.PIVOT_IN));
                break;
            case PIVOT_OUT:
                pivotMotor.setControl(motorRequest.withPosition(FeederAndIntakeConstants.PIVOT_OUT));
                break;
        }
    }

    public boolean atSetpoint() {
        return (Math.abs(pivotPosition.getValueAsDouble() - motorRequest.Position) < 0.75);
    }

    @Override
    public void periodic() {
        // // Refresh all of the signals for processing.
        // BaseStatusSignal.refreshAll(
        //     pivotPosition,
        //     pivotStatorCurrent,
        //     pivotSupplyCurrent
        // );

        // // Pivot encoder.
        // DogLog.log("Intake/Pivot/Position", pivotPosition.getValueAsDouble(), Rotations);
        // DogLog.log("Intake/Pivot/Velocity", pivotVelocity.getValueAsDouble(), RotationsPerSecond);

        // // Pivot currents.
        // DogLog.log("Intake/Pivot/StatorCurrent", pivotStatorCurrent.getValueAsDouble(), Amps);
        // DogLog.log("Intake/Pivot/SupplyCurrent", pivotSupplyCurrent.getValueAsDouble(), Amps);
    }
}
