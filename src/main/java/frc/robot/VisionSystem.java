package frc.robot;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.livoniawarriors.odometry.Odometry;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase {
    Odometry odometry;
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator frontCamEstimator;
    private VisionSystemSim visionSim;
    private boolean simInit;

    private PhotonCamera frontCam;
    private Transform3d frontCamPos;

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVIATIONS = VecBuilder.fill(4.0, 4.0, 8.0);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVIATIONS = VecBuilder.fill(0.5, 0.5, 1.0);
    public static final double MAX_VISION_DISTANCE = 4.0;

    public VisionSystem(Odometry odometry) {
        super();
        this.odometry = odometry;
        simInit = false;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            // should never fail, as WpiLib always provides this file
        }

        // get camera by name
        frontCam = new PhotonCamera("FrontCam");
        // get the offsets where the camera is mounted
        frontCamPos = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0.0, 0.0, 0.0));
        // get the estimator of it
        frontCamEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                frontCam, frontCamPos);
    }

    @Override
    public void periodic() {
        frontCamEstimator.setReferencePose(odometry.getPose());
        Optional<EstimatedRobotPose> frontPose = frontCamEstimator.update();
        frontCam.getLatestResult();
        if (frontPose.isPresent()) {
            EstimatedRobotPose pose = frontPose.get();
            Matrix<N3, N1> deviations = getEstimationStdDeviations(pose.estimatedPose.toPose2d(), frontCamEstimator,
                    frontCam.getLatestResult());
            odometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, deviations);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(aprilTagFieldLayout);
            // Create simulated camera properties. These can be set to mimic your actual
            // camera.
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(111.0));
            cameraProp.setCalibError(0.37, 0.13);
            cameraProp.setFPS(15.0);
            cameraProp.setAvgLatencyMs(50.0);
            cameraProp.setLatencyStdDevMs(15.0);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible
            // targets.
            PhotonCameraSim cameraSim = new PhotonCameraSim(frontCam, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, frontCamPos);
            // if you want to see a virtual camera, set this to true, and go to
            // http://localhost:1182/
            cameraSim.enableDrawWireframe(true);
            simInit = true;
        }
        visionSim.update(odometry.getPose());
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDeviations(Pose2d estimatedPose, PhotonPoseEstimator photonEstimator,
            PhotonPipelineResult result) {
        Matrix<N3, N1> estStdDeviations;

        List<PhotonTrackedTarget> targets = result.getTargets();
        int numTags = 0;
        double avgDist = 0.0;
        for (PhotonTrackedTarget tgt : targets) {
            Optional<Pose3d> tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0) {
            // no tags
            estStdDeviations = SINGLE_TAG_STD_DEVIATIONS;
        } else if (numTags == 1 && avgDist > MAX_VISION_DISTANCE) {
            // one tag, but too far. Making these large so they will be ignored
            estStdDeviations = VecBuilder.fill(1e100, 1e100, 1e100);
        } else if (numTags == 1) {
            // one tag close
            estStdDeviations = SINGLE_TAG_STD_DEVIATIONS;
        } else {
            // multiple tags seen
            avgDist /= numTags;
            estStdDeviations = MULTI_TAG_STD_DEVIATIONS;
        }

        return estStdDeviations.times(1 + (avgDist * avgDist / 30));
    }
}
