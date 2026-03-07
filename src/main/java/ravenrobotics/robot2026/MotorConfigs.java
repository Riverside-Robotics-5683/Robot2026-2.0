package ravenrobotics.robot2026;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import ravenrobotics.robot2026.Constants.FeederAndIntakeConstants;
import ravenrobotics.robot2026.Constants.FlywheelAndHoodConstants;

public class MotorConfigs {
    public static final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    public static final SparkFlexConfig feederConfig = new SparkFlexConfig();
    public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    public static final SparkFlexConfig columnConfig = new SparkFlexConfig();

    static {
        pivotConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        pivotConfig.CurrentLimits
            .withStatorCurrentLimit(50)
            .withSupplyCurrentLimit(50);
        pivotConfig.Slot0
            .withKP(FeederAndIntakeConstants.PIVOT_KP)
            .withKI(FeederAndIntakeConstants.PIVOT_KI)
            .withKD(FeederAndIntakeConstants.PIVOT_KD);

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
            .p(FlywheelAndHoodConstants.FLYWHEEL_KP)
            .i(FlywheelAndHoodConstants.FLYWHEEL_KI)
            .d(FlywheelAndHoodConstants.FLYWHEEL_KD);

        columnConfig.smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
    }
}
