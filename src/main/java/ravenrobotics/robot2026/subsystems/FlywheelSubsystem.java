package ravenrobotics.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.MotorConfigs;
import ravenrobotics.robot2026.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
    
    private final SparkFlex leftFlywheel, centerFlywheel, rightFlywheel;

    private final TalonFX columnMotor;

    private final RelativeEncoder centerFlywheelEncoder;
    private final SparkClosedLoopController centerFlywheelController;

    private final StatusSignal<AngularVelocity> columnVelocitySignal;
    private final StatusSignal<Current> columnSupplyCurrentSignal;
    private final StatusSignal<Current> columnStatorCurrentSignal;

    public double flywheelSpeed = 3000.0;

    private double commandedSetpoint = 0;
    private boolean isIdle = true;

    public enum FlywheelState {
        FLYWHEEL_STOP,
        FLYWHEEL_IDLE,
        FLYWHEEL_RUN
    }

    public enum FlywheelIdleState {
        HUB,
        PASS
    }

    public FlywheelSubsystem() {
        leftFlywheel = new SparkFlex(FlywheelConstants.LEFT_FLYWHEEL_MOTOR, MotorType.kBrushless);
        centerFlywheel = new SparkFlex(FlywheelConstants.CENTER_FLYWHEEL_MOTOR, MotorType.kBrushless);
        rightFlywheel = new SparkFlex(FlywheelConstants.RIGHT_FLYWHEEL_MOTOR, MotorType.kBrushless);

        centerFlywheel.configure(MotorConfigs.flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFlywheel.configure(MotorConfigs.flywheelConfig.follow(centerFlywheel, false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFlywheel.configure(MotorConfigs.flywheelConfig.follow(centerFlywheel, false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        centerFlywheelEncoder = centerFlywheel.getEncoder();
        centerFlywheelController = centerFlywheel.getClosedLoopController();

        columnMotor = new TalonFX(FlywheelConstants.COLUMN_MOTOR);

        columnMotor.getConfigurator().apply(MotorConfigs.columnConfig);

        columnVelocitySignal = columnMotor.getVelocity();

        columnStatorCurrentSignal = columnMotor.getStatorCurrent();
        columnSupplyCurrentSignal = columnMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            columnVelocitySignal,
            columnStatorCurrentSignal,
            columnSupplyCurrentSignal);

        columnMotor.optimizeBusUtilization();

        DogLog.tunable("Flywheel/ManualSpeed", flywheelSpeed, (newSpeed) -> {
            this.flywheelSpeed = newSpeed;
        });

        this.register();
    }

    public void runFlywheel(double speed) {
        isIdle = false;
        commandedSetpoint = speed;
        centerFlywheelController.setSetpoint(speed, ControlType.kVelocity);
    }

    public void idleFlywheel(FlywheelIdleState state) {
        isIdle = true;
        switch (state) {
            case HUB:
                commandedSetpoint = FlywheelConstants.FLYWHEEL_HUB_IDLE;
                centerFlywheelController.setSetpoint(FlywheelConstants.FLYWHEEL_HUB_IDLE, ControlType.kVelocity);
                break;
            case PASS:
                commandedSetpoint = FlywheelConstants.FLYWHEEL_PASS_IDLE;
                centerFlywheelController.setSetpoint(FlywheelConstants.FLYWHEEL_PASS_IDLE, ControlType.kVelocity);
                break;
        }
    }

    public void stopFlywheel() {
        centerFlywheel.stopMotor();
    }

    public boolean atSetpoint() {
        return !isIdle && (Math.abs(commandedSetpoint - centerFlywheelEncoder.getVelocity()) < FlywheelConstants.FLYWHEEL_SETPOINT_TOLERANCE);
    }

    public void runColumn(boolean isReverse) {
        columnMotor.set(isReverse ? -1 : 1);
    }

    public void stopColumn() {
        columnMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double flywheelVelocity;
        double leftFlywheelCurrent, centerFlywheelCurrent, rightFlywheelCurrent;

        double columnVelocity, columnSupplyCurrent, columnStatorCurrent;

        BaseStatusSignal.refreshAll(
            columnStatorCurrentSignal,
            columnSupplyCurrentSignal,
            columnVelocitySignal);

        columnVelocity = columnVelocitySignal.getValueAsDouble();

        columnSupplyCurrent = columnSupplyCurrentSignal.getValueAsDouble();
        columnStatorCurrent = columnStatorCurrentSignal.getValueAsDouble();

        flywheelVelocity = centerFlywheelEncoder.getVelocity();

        leftFlywheelCurrent = leftFlywheel.getOutputCurrent();
        centerFlywheelCurrent = centerFlywheel.getOutputCurrent();
        rightFlywheelCurrent = rightFlywheel.getOutputCurrent();

        DogLog.log("Flywheel/Velocity", flywheelVelocity, RPM);

        DogLog.log("Flywheel/Current/Left", leftFlywheelCurrent, Amps);
        DogLog.log("Flywheel/Current/Center", centerFlywheelCurrent, Amps);
        DogLog.log("Flywheel/Current/Right", rightFlywheelCurrent, Amps);

        DogLog.log("Flywheel/Column/Velocity", columnVelocity, RPM);
        DogLog.log("Flywheel/Column/SupplyCurrent", columnSupplyCurrent, Amps);
        DogLog.log("Flywheel/Column/StatorCurrent", columnStatorCurrent, Amps);

        DogLog.log("Flywheel/AtSetpoint", atSetpoint());
    }
}
