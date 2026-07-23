package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

public class VisionSubsystem extends SubsystemBase
{

    private final PhotonCamera camera = new PhotonCamera("bodyCam");
    private final PhotonPoseEstimator poseEstimator;

    public VisionSubsystem()
    {
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Camera is mounted 0.381m forward, 0.0508m left, 0.3683m up from robot center, facing the robot's front (yaw offset by PI since the camera faces opposite the mount's zero direction).
        Transform3d robotToCamera = new Transform3d(new Translation3d(0.381,0.0508,0.3683), new Rotation3d(0.0, 0.0, Math.PI));

        poseEstimator = new PhotonPoseEstimator(fieldLayout,PoseStrategy.LOWEST_AMBIGUITY,robotToCamera);
    }

    public Optional<EstimatedRobotPose> getEstimatedPose()
    {

        var result = camera.getLatestResult();

        if(!result.hasTargets())
        {
            return Optional.empty();
        }

        return poseEstimator.update(result);
    }

    @Override
    public void periodic()
    {

    }

}
