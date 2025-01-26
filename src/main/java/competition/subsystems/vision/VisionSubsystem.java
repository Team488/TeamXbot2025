package competition.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.kobe.xbot.JClient.XTablesClient;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCameraExtended;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.injection.electrical_contract.XCameraElectricalContract;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.logic.TimeStableValidator;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;
import xbot.common.subsystems.vision.AprilTagCamera;
import xbot.common.subsystems.vision.CameraCapabilities;
import xbot.common.subsystems.vision.SimpleCamera;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

@Singleton
public class VisionSubsystem extends BaseSubsystem implements DataFrameRefreshable {

    final RobotAssertionManager assertionManager;
    final BooleanProperty isInverted;
    final DoubleProperty yawOffset;
    final DoubleProperty waitForStablePoseTime;
    final DoubleProperty robotDisplacementThresholdToRejectVisionUpdate;
    final DoubleProperty singleTagStableDistance;
    final DoubleProperty multiTagStableDistance;

    // xtables properties
    final StringProperty xtablesCoordinateLocation;
    final StringProperty xtablesHeadingLocation;

    // always persisted xtables instance
    private XTablesClient xclient;

    AprilTagFieldLayout aprilTagFieldLayout;
    final ArrayList<AprilTagCamera> aprilTagCameras;
    final ArrayList<SimpleCamera> allCameras;
    boolean aprilTagsLoaded = false;
    long logCounter = 0;


    @Inject
    public VisionSubsystem(PropertyFactory pf, XCameraElectricalContract electricalContract, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;

        pf.setPrefix(this);
        isInverted = pf.createPersistentProperty("Yaw inverted", true);
        yawOffset = pf.createPersistentProperty("Yaw offset", 0);
        singleTagStableDistance = pf.createPersistentProperty("Single tag stable distance", 2.0);
        multiTagStableDistance = pf.createPersistentProperty("Multi tag stable distance", 4.0);

        xtablesCoordinateLocation = pf.createPersistentProperty("Xtables Coordinate Location", "target_waypoints");
        xtablesHeadingLocation = pf.createPersistentProperty("Xtables Heading Location", "target_heading");
        var trackingNt = NetworkTableInstance.getDefault().getTable("SmartDashboard");

        waitForStablePoseTime = pf.createPersistentProperty("Pose stable time", 0.0, Property.PropertyLevel.Debug);
        robotDisplacementThresholdToRejectVisionUpdate = pf.createPersistentProperty("Displacement Threshold to reject Vision (m)",3);

        // TODO: Add resiliency to this subsystem, so that if the camera is not connected, it doesn't cause a pile
        // of errors. Some sort of VisionReady in the ElectricalContract may also make sense. Similarly,
        // we need to handle cases like not having the AprilTag data loaded.

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            aprilTagsLoaded = true;
            log.info("Successfully loaded AprilTagFieldLayout");
        } catch (IOException e) {
            log.error("Could not load AprilTagFieldLayout!", e);
        }

        PhotonCameraExtended.setVersionCheckEnabled(false);
        aprilTagCameras = new ArrayList<AprilTagCamera>();
        if (aprilTagsLoaded) {
            var aprilTagCapableCameras = Arrays
                    .stream(electricalContract.getCameraInfo())
                    .filter(info -> info.capabilities().contains(CameraCapabilities.APRIL_TAG))
                    .toArray(CameraInfo[]::new);
            for (var camera : aprilTagCapableCameras) {
                aprilTagCameras.add(new AprilTagCamera(camera, waitForStablePoseTime::get, aprilTagFieldLayout, this.getPrefix()));
            }
        }

        allCameras = new ArrayList<>();
        allCameras.addAll(aprilTagCameras);

        xclient = new XTablesClient();
    }

    public XTablesClient getXTablesClient(){
        // in case of any weirdness
        if(xclient == null){
            xclient = new XTablesClient();
        }
        return xclient;
    }

    public String getXtablesCoordinateLocation(){
        return xtablesCoordinateLocation.get();
    }

    public String getXtablesHeadingLocation(){
        return xtablesHeadingLocation.get();
    }

    public List<Optional<EstimatedRobotPose>> getPhotonVisionEstimatedPoses(Pose2d previousEstimatedRobotPose) {
        var estimatedPoses = new ArrayList<Optional<EstimatedRobotPose>>();
        for (AprilTagCamera cameraState : this.aprilTagCameras) {
            if (cameraState.isCameraWorking()) {
                var estimatedPose = getPhotonVisionEstimatedPose(cameraState.getName(), cameraState.getPoseEstimator(),
                        previousEstimatedRobotPose, cameraState.getIsStableValidator(), cameraState.getCamera());
                estimatedPoses.add(estimatedPose);
            }
        }
        return estimatedPoses;
    }

    public Optional<EstimatedRobotPose> getPhotonVisionEstimatedPose(
            String name, PhotonPoseEstimator estimator, Pose2d previousEstimatedRobotPose, TimeStableValidator poseTimeValidator,
            PhotonCameraExtended photonCameraExtended) {
        if (aprilTagsLoaded) {
            estimator.setReferencePose(previousEstimatedRobotPose);

            PhotonPipelineResult cameraResult = photonCameraExtended.getAllUnreadResults().get(photonCameraExtended.getAllUnreadResults().size() - 1);


            var estimatedPose = estimator.update(cameraResult, photonCameraExtended.getCameraMatrix(), photonCameraExtended.getDistCoeffs());
            // Log the estimated pose, and log an insane value if we don't have one (so we don't clutter up the visualization)
            if (estimatedPose.isPresent())
            {
                aKitLog.record(name+"Estimate", estimatedPose.get().estimatedPose.toPose2d());
            }

            var isReliable = estimatedPose.isPresent() && isEstimatedPoseReliable(estimatedPose.get(), previousEstimatedRobotPose);
            aKitLog.record(name+"Reliable", isReliable);
            var isStable = waitForStablePoseTime.get() == 0.0 || poseTimeValidator.checkStable(isReliable);
            if (isReliable && isStable) {
                return estimatedPose;
            }
            return Optional.empty();
        } else {
            return Optional.empty();
        }
    }

    /**
     * Performs several sanity checks on the estimated pose:
     * - Are we seeing an invalid tag?
     * - Are we pretty close to our previous location?
     * - If we see 2+ targets, we're good
     * - If we see 1 target, we need to be close and have a low ambiguity
     * @param estimatedPose The pose to check
     * @param previousEstimatedPose The previous location of the robot in the last loop
     * @return True if the pose is reliable and should be consumed by the robot
     */
    public boolean isEstimatedPoseReliable(EstimatedRobotPose estimatedPose, Pose2d previousEstimatedPose) {
        // No targets, so there's no way we should use this data.
        if (estimatedPose.targetsUsed.size() == 0) {
            return false;
        }

        // Pose isn't reliable if we see a tag id that shouldn't be on the field
        var allTagIds = getTagListFromPose(estimatedPose);
        if (allTagIds.stream().anyMatch(id -> id < 1 || id > 16)) {
            log.warn("Ignoring vision pose with invalid tag id. Visible tags: "
                    + getStringFromList(allTagIds));
            return false;
        }

        // If this is way too far from our current location, then it's probably a bad estimate.
        double distance = previousEstimatedPose.getTranslation().getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
        var visionDistanceTooLarge = distance > robotDisplacementThresholdToRejectVisionUpdate.get();
        // Unless we're near zero which means we've never set a pose before, use vision even if we're far away

        var isNearZero = previousEstimatedPose.getTranslation().getNorm() < 0.1;
        if(!isNearZero && visionDistanceTooLarge) {
            if (logCounter++ % 20 == 0) {
                log.warn(String.format("Ignoring vision pose because distance is %f from our previous pose. Current pose: %s, vision pose: %s.",
                        distance,
                        previousEstimatedPose.getTranslation().toString(),
                        estimatedPose.estimatedPose.getTranslation().toString()));
            }
            return false;
        }

        // How far away is the camera from the target?
        double cameraDistanceToTarget =
                estimatedPose.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm();

        // Two or more targets tends to be very reliable, but there's still a limit for distance
        if (estimatedPose.targetsUsed.size() > 1
                && cameraDistanceToTarget < multiTagStableDistance.get()) {
            return true;
        }

        // For a single target we need to be above reliability threshold and very close.
        return estimatedPose.targetsUsed.get(0).getPoseAmbiguity() < 0.20
                && cameraDistanceToTarget < singleTagStableDistance.get();
    }

    private List<Integer> getTagListFromPose(EstimatedRobotPose estimatedPose) {
        return Arrays.asList(estimatedPose.targetsUsed.stream()
                .map(target -> target.getFiducialId()).toArray(Integer[]::new));
    }

    private String getStringFromList(List<Integer> list) {
        return String.join(", ", list.stream().mapToInt(id -> id).mapToObj(id -> Integer.toString(id)).toArray(String[]::new));
    }

    int loopCounter = 0;

    @Override
    public void periodic() {
        loopCounter++;

        var anyCameraBroken = allCameras.stream().anyMatch(state -> !state.isCameraWorking());

        // If one of the cameras is not working, see if they have self healed every 5 seconds
        if (loopCounter % (50 * 5) == 0 && (anyCameraBroken)) {
            log.info("Checking if cameras have self healed");
            for (SimpleCamera camera : aprilTagCameras) {
                if (!camera.isCameraWorking()) {
                    log.info("Camera " + camera.getName() + " is still not working");
                }
            }
        }

        for (SimpleCamera camera : allCameras) {
            aKitLog.record(camera.getName() + "CameraWorking", camera.isCameraWorking());
        }
    }

    @Override
    public void refreshDataFrame() {
        if (aprilTagsLoaded) {
            for (SimpleCamera camera : allCameras) {
                camera.getCamera().refreshDataFrame();
            }
        }
    }

    public int cameraWorkingState() {
        if (allCameras.stream().allMatch(state -> state.isCameraWorking())) {
            // If all are working, return 0
            return 0;
        }
        else if (allCameras.stream().allMatch(state -> !state.isCameraWorking())) {
            // If no cameras are working, return 1
            return 1;
        }
        // If some of the cameras are working, return 2
        return 2;
    }
}