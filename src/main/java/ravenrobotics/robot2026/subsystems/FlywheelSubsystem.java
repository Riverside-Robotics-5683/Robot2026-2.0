package ravenrobotics.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.MotorConfigs;
import ravenrobotics.robot2026.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
    
    private final SparkFlex leftFlywheel, centerFlywheel, rightFlywheel;

    private final SparkFlex columnMotor;

    private final RelativeEncoder centerFlywheelEncoder;
    private final SparkClosedLoopController centerFlywheelController;

    private final RelativeEncoder columnEncoder;

    public double flywheelSpeed = 3000.0;

    private boolean isIdle = true;

    public enum FlywheelState {
        FLYWHEEL_STOP,
        FLYWHEEL_IDLE,
        FLYWHEEL_RUN
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

        columnMotor = new SparkFlex(FlywheelConstants.COLUMN_MOTOR, MotorType.kBrushless);
        columnMotor.configure(MotorConfigs.columnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        columnEncoder = columnMotor.getEncoder();

        DogLog.tunable("Flywheel/ManualSpeed", flywheelSpeed, (newSpeed) -> {
            this.flywheelSpeed = newSpeed;
        });

        this.register();
    }

    public void runFlywheel(double speed) {
        isIdle = false;
        centerFlywheelController.setSetpoint(speed, ControlType.kVelocity);
    }

    public void idleFlywheel() {
        isIdle = true;
        centerFlywheelController.setSetpoint(FlywheelConstants.FLYWHEEL_IDLE, ControlType.kVelocity);
    }

    public void stopFlywheel() {
        centerFlywheel.stopMotor();
    }

    public boolean atSetpoint() {
        return !isIdle && (Math.abs(centerFlywheelController.getSetpoint() - centerFlywheelEncoder.getVelocity()) < 400);
    }

    public void runColumn(boolean isReverse) {
        if (isReverse) {
            columnMotor.set(-1);
        } else {
            columnMotor.set(1);
        }
    }

    public void stopColumn() {
        columnMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double flywheelVelocity;
        double leftFlywheelCurrent, centerFlywheelCurrent, rightFlywheelCurrent;

        double columnVelocity, columnCurrent;

        flywheelVelocity = centerFlywheelEncoder.getVelocity();

        leftFlywheelCurrent = leftFlywheel.getOutputCurrent();
        centerFlywheelCurrent = centerFlywheel.getOutputCurrent();
        rightFlywheelCurrent = rightFlywheel.getOutputCurrent();

        columnVelocity = columnEncoder.getVelocity();
        columnCurrent = columnMotor.getOutputCurrent();

        DogLog.log("Flywheel/Velocity", flywheelVelocity, RPM);

        DogLog.log("Flywheel/Current/Left", leftFlywheelCurrent, Amps);
        DogLog.log("Flywheel/Current/Center", centerFlywheelCurrent, Amps);
        DogLog.log("Flywheel/Current/Right", rightFlywheelCurrent, Amps);

        DogLog.log("Flywheel/Column/Velocity", columnVelocity, RPM);
        DogLog.log("Flywheel/Column/Current", columnCurrent, Amps);

        DogLog.log("Flywheel/AtSetpoint", atSetpoint());
    }
}
