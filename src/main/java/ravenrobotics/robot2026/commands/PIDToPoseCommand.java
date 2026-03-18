package ravenrobotics.robot2026.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot2026.Constants.AutoConstants;
import ravenrobotics.robot2026.subsystems.CommandSwerveDrivetrain;

/**
 * Command for going to a specified pose with PID.
 */
public class PIDToPoseCommand extends Command {

    private final CommandSwerveDrivetrain driveSubsystem;

    private final Pose2d targetPose;

    private final PhoenixPIDController xController = new PhoenixPIDController(5.0, 0, 0);
    private final PhoenixPIDController yController = new PhoenixPIDController(5.0, 0, 0);
    private final PhoenixPIDController thetaController = new PhoenixPIDController(1.0, 0, 0);

    private RobotCentric driveRequest = new RobotCentric();

    public PIDToPoseCommand(Pose2d targetPose, CommandSwerveDrivetrain driveSubsystem) {
        this.targetPose = targetPose;
        this.driveSubsystem = driveSubsystem;

        addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        double currentTimestamp = Timer.getTimestamp();
        Pose2d currentPosition = driveSubsystem.getState().Pose;

        double xValue, yValue, thetaValue;

        xValue = xController.calculate(currentPosition.getX(), targetPose.getX(), currentTimestamp) * AutoConstants.AUTO_TRANSLATION_SPEED;
        yValue = yController.calculate(currentPosition.getY(), targetPose.getY(), currentTimestamp) * AutoConstants.AUTO_TRANSLATION_SPEED;
        thetaValue = thetaController.calculate(
            currentPosition.getRotation().getRadians(),
            targetPose.getRotation().getRadians(),
            currentTimestamp);

        thetaValue *= AutoConstants.AUTO_ROTATION_SPEED;

        driveSubsystem.setControl(
            driveRequest
                .withVelocityX(xValue)
                .withVelocityY(yValue)
                .withRotationalRate(thetaValue)
        );
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}
