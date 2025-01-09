package frc.robot.subsystems.vision.estimator;

import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public record VisionUpdate(Pose3d estimatedPose, double timestamp, List<PhotonTrackedTarget> targetsUsed) {}
