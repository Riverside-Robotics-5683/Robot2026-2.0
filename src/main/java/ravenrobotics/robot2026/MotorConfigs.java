package ravenrobotics.robot2026;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import ravenrobotics.robot2026.Constants.PivotConstants;
import ravenrobotics.robot2026.Constants.FlywheelConstants;

public class MotorConfigs {
    public static final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    public static final SparkFlexConfig feederConfig = new SparkFlexConfig();
    public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    public static final SparkFlexConfig columnConfig = new SparkFlexConfig();

    public static final SparkMaxConfig actuatorConfig = new SparkMaxConfig();

    static {
        pivotConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        pivotConfig.CurrentLimits
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(20);
        pivotConfig.Slot0
            .withKP(PivotConstants.PIVOT_KP)
            .withKI(PivotConstants.PIVOT_KI)
            .withKD(PivotConstants.PIVOT_KD);
        pivotConfig.ClosedLoopRamps
            .withVoltageClosedLoopRampPeriod(0.75)
            .withTorqueClosedLoopRampPeriod(0.75)
            .withDutyCycleClosedLoopRampPeriod(0.75);

        intakeConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        intakeConfig.CurrentLimits
            .withStatorCurrentLimit(50)
            .withSupplyCurrentLimit(50);

        feederConfig.smartCurrentLimit(50).idleMode(IdleMode.kCoast).inverted(true);

        flywheelConfig.smartCurrentLimit(50)
            .closedLoopRampRate(2)
            .idleMode(IdleMode.kCoast)
            .inverted(true);
        flywheelConfig.closedLoop.outputRange(-1, 1)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(FlywheelConstants.FLYWHEEL_KP)
            .i(FlywheelConstants.FLYWHEEL_KI)
            .d(FlywheelConstants.FLYWHEEL_KD);

        columnConfig.smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast)
            .inverted(false);

        actuatorConfig.smartCurrentLimit(1, 1)
            .inverted(false)
            .closedLoopRampRate(1)
            .idleMode(IdleMode.kCoast);

        actuatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .outputRange(-1, 1)
            .p(1.0)
            .i(0.0)
            .d(0.0);
    }
}
