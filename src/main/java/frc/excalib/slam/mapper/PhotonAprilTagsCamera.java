package frc.excalib.slam.mapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Annotations;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;
import java.util.function.BiConsumer;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

public class PhotonAprilTagsCamera implements Logged {
    private final PhotonCamera m_camera;
    private final AprilTagFieldLayout m_fieldLayout;

    private final PhotonPoseEstimator m_photonPoseEstimator;

    private final Transform3d kRobotToCamera;

    private final double TOO_FAR = 3.5; // m
    private final double TOO_LONG = 3; // s
    private final Timer tagTimer = new Timer();
    private final Timer tagTimerHigh = new Timer();
    private final Trigger seenTrigger;
    private final Trigger seenTriggerHigh;

    public PhotonAprilTagsCamera(String cameraName, AprilTagFields aprilTagField, Transform3d robotToCamera) {
        m_camera = new PhotonCamera(cameraName);
        m_camera.setDriverMode(false);

        tagTimer.start();
        tagTimerHigh.start();
        seenTrigger = new Trigger(()-> tagTimer.get() < TOO_LONG);
        seenTriggerHigh = new Trigger(()-> tagTimerHigh.get() < TOO_LONG);

        m_fieldLayout = AprilTagFieldLayout.loadField(aprilTagField);

        kRobotToCamera = robotToCamera;
        m_photonPoseEstimator = new PhotonPoseEstimator(m_fieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamera);
        m_photonPoseEstimator.setMultiTagFallbackStrategy(LOWEST_AMBIGUITY);
    }

    public PhotonCamera getCamera() {
        return m_camera;
    }

    public void setDriverMode(boolean isDriverMode) {
        m_camera.setDriverMode(isDriverMode);
    }

    public void setPipeline(int index) {
        m_camera.setPipelineIndex(index);
    }

    public PhotonPipelineResult getLatestResualt() {
        var result = m_camera.getLatestResult();

        if (result != null) return result;
        return new PhotonPipelineResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        PhotonPipelineResult result = m_camera.getLatestResult();

        if (result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < 0.2) {
            Translation2d targetTranslation = result.getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d();
            if (targetTranslation.getDistance(new Translation2d(0, 0)) < TOO_FAR) {
                tagTimer.restart();
                m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
                return m_photonPoseEstimator.update(result);
            }
        }

        return Optional.empty();
    }

    public boolean updateFromAprilTagPose(BiConsumer<Pose2d, Double> toUpdate) {
        var result = m_camera.getLatestResult();
        if (!result.hasTargets()) return false;

        var id = result.getBestTarget().getFiducialId();
        if (id == -1) return false;

        var tag = m_fieldLayout.getTagPose(id);
        if (tag.isEmpty()) return false;

        toUpdate.accept(tag.get().plus(result.getBestTarget().getBestCameraToTarget()).toPose2d(), result.getTimestampSeconds());
        return true;
    }

    @Log.NT
    public boolean getTagTimer(){
        return seenTrigger.getAsBoolean();
    }
    @Log.NT
    public boolean getTagTimerHigh(){
        return seenTriggerHigh.getAsBoolean();
    }
}

