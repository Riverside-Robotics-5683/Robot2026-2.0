package ravenrobotics.robot2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.MotorConfigs;
import ravenrobotics.robot2026.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {
    private final SparkMax leftActuator;
    private final SparkMax rightActuator;

    private final AbsoluteEncoder hoodEncoder;
    private final SparkClosedLoopController rightController;

    private double targetPosition = 0.0;

    public HoodSubsystem() {
        leftActuator = new SparkMax(HoodConstants.LEFT_ACTUATOR, MotorType.kBrushed);
        rightActuator = new SparkMax(HoodConstants.RIGHT_ACTUATOR, MotorType.kBrushed);

        rightActuator.configure(MotorConfigs.actuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftActuator.configure(MotorConfigs.actuatorConfig.follow(rightActuator), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodEncoder = rightActuator.getAbsoluteEncoder();
        rightController = rightActuator.getClosedLoopController();
    }

    public void setPosition(double position) {
        targetPosition = position;

        rightController.setSetpoint(targetPosition, ControlType.kPosition);
    }

    public void testSetPosition(double position) {
        targetPosition = position;

        double currentPosition = hoodEncoder.getPosition();

        if (currentPosition < position) {
            rightActuator.set(0.75);
        } else if (currentPosition > position) {
            rightActuator.set(-0.75);
        } else {
            stopActuators();
        }
    }

    public void runActuators(boolean reverse) {
        if (reverse) {
            rightActuator.set(-1);
        } else {
            rightActuator.set(1);
        }
    }

    public boolean atSetpoint() {
        return rightController.isAtSetpoint();
    }

    public boolean testAtSetpoint() {
        return Math.abs(hoodEncoder.getPosition() - targetPosition) < 0.01;
    }

    public void stopActuators() {
        leftActuator.stopMotor();
        rightActuator.stopMotor();
    }

    @Override
    public void periodic() {
        double encoderPosition, encoderVelocity, leftCurrent, rightCurrent;

        encoderPosition = hoodEncoder.getPosition();
        encoderVelocity = hoodEncoder.getVelocity();

        leftCurrent = leftActuator.getOutputCurrent();
        rightCurrent = rightActuator.getOutputCurrent();

        DogLog.log("Hood/Position", encoderPosition, Rotations);
        DogLog.log("Hood/TargetPosition", targetPosition, Rotations);
        DogLog.log("Hood/Velocity", encoderVelocity, RPM);

        DogLog.log("Hood/LeftCurrent", leftCurrent, Amps);
        DogLog.log("Hood/RightCurrent", rightCurrent, Amps);
    }
}
