package ravenrobotics.robot2026.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera flywheelCamera;

    public VisionSubsystem() {
        flywheelCamera = new PhotonCamera(VisionConstants.FLYWHEEL_CAMERA);

        this.register();
    }
}
