package ravenrobotics.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.Constants.FeederAndIntakeConstants;
import ravenrobotics.robot2026.MotorConfigs;

/**
 * Subsystem for controlling the feeder mecahnism.
 */
public class FeederSubsystem extends SubsystemBase {

    // Feeder motor.
    private final SparkFlex feederMotor;

    // Feeder encoder.
    private final RelativeEncoder feederEncoder;

    /**
     * Directions the feeder can be commanded to.
     */
    public enum FeederDirection {
        /**
         * Moves the feeder towards the column.
         */
        FEEDER_IN,
        /**
         * Moves the feeder towards the intake.
         */
        FEEDER_OUT,
        /**
         * Stops the feeder.
         */
        FEEDER_STOP
    }

    /**
     * Constructs the subsystem for controlling the feeder.
     */
    public FeederSubsystem() {
        // Initialize feeder motor.
        feederMotor = new SparkFlex(FeederAndIntakeConstants.FEEDER_MOTOR, MotorType.kBrushless);

        // Configure feeder motor.
        feederMotor.configure(MotorConfigs.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get relative encoder for feeder.
        feederEncoder = feederMotor.getEncoder();

        this.register();
    }
    
    public void setFeeder(FeederDirection direction) {
        switch (direction) {
            case FEEDER_IN:
                feederMotor.set(1);
                break;
            case FEEDER_OUT:
                feederMotor.set(-1);
                break;
            case FEEDER_STOP:
                feederMotor.stopMotor();
                break;
        }
    }

    @Override
    public void periodic() {
        double feedVelocity = feederEncoder.getVelocity();
        double feedCurrent = feederMotor.getOutputCurrent();

        // Feeder encoder.
        DogLog.log("Feeder/Velocity", feedVelocity, RPM);

        // Feeder current.
        DogLog.log("Feeder/OutputCurrent", feedCurrent, Amps);
    }
}
