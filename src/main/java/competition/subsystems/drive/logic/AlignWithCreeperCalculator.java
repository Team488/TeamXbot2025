package competition.subsystems.drive.logic;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.kobe.xbot.JClient.CachedSubscriber;
import org.kobe.xbot.JClient.XTablesClient;
import xbot.common.advantage.AKitLogger;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;

import javax.inject.Inject;


public class AlignWithCreeperCalculator {
    private boolean initalized = false;
    private final Logger log;
    private final AKitLogger aKitLog;
    private final PIDManager pidManager;

    final DriveSubsystem drive;
    final CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem;

    private final int tunedWidth = 960; // what resolution the pid tuning was done at
    private final int tunedHeight = 720; // what resolution the pid tuning was done at

    private final String tableLeftDistance = "verticalEdgeLeftDistancePx";
    private final String tableRightDistance = "verticalEdgeRightDistancePx";
    private final String tableHres = "cameraHres";
    private final String tableVres = "cameraVres";

    private final StringProperty photonVisionFrontLeftHostname;
    private final StringProperty photonVisionFrontRightHostname;

    private final DoubleProperty driveGain;
    private final DoubleProperty errorSlope;
    private final DoubleProperty maxError;
    private final DoubleProperty errorThresholdPixels;
    private final DoubleProperty pushForce;

    private CachedSubscriber leftOffsetPixelsSubscriber;
    private CachedSubscriber rightOffsetPixelsSubscriber;
    private CachedSubscriber hresSubscriber;
    private CachedSubscriber vresSubscriber;


    private String hostname;
    private int camera;

    // set defaults for now
    private int currentCamHres = this.tunedWidth;
    private int currentCamVres = this.tunedHeight;
    private boolean forceStop = false;

    private Boolean isCenteredConfidently = false;

    @Inject
    public AlignWithCreeperCalculator(DriveSubsystem drive,
                                      CoprocessorCommunicationSubsystem coprocessorCommunications,
                                      PoseSubsystem pose,
                                      PropertyFactory pf,
                                      PIDManager.PIDManagerFactory pidFactory) {
        this.log = LogManager.getLogger("AlignWithCreeperCommmand");
        this.aKitLog = new AKitLogger("AlignWithCreeperCommmand/");
        this.drive = drive;
        this.pidManager = pidFactory.create("AlignWithCreeperCommand", 0.6, 0.0001, 0.9);
        this.coprocessorCommunicationSubsystem = coprocessorCommunications;

        pf.setPrefix("AlignWithCreeperCommand/");
        this.driveGain = pf.createPersistentProperty("Drive Gain", 0.18);
        this.errorSlope = pf.createPersistentProperty("Cost Function Error slope", 1);
        this.maxError = pf.createPersistentProperty("Max Error", 0.2);
        this.photonVisionFrontLeftHostname = pf.createPersistentProperty(
                "Photon Vision Front Left Hostname", "photonvisionfrontleft");
        this.photonVisionFrontRightHostname = pf.createPersistentProperty(
                "Photon Vision Front Right Hostname", "photonvisionfrontright");
        this.errorThresholdPixels = pf.createPersistentProperty(
                "Pixel error Threshold", 35);
        this.pushForce = pf.createPersistentProperty("Creeper push force", 0.05);
    }


    public boolean initialize() {
        this.pidManager.reset();
        // reset flags
        forceStop = false;
        isCenteredConfidently = false;

        // try get xtables client
        XTablesClient client =
                this.coprocessorCommunicationSubsystem.tryGetXTablesClient();

        if (client == null) {
            log.warn("Failed to obtain a valid XTablesClient from CoprocessorCommunicationSubsystem. "
                    + "Aborting initialization!");
            return false;
        }

        // Determine active camera and retrieve its corresponding resolution and hostname.
        if (camera == Cameras.FRONT_LEFT_CAMERA.getIndex()) {
            this.hostname = photonVisionFrontLeftHostname.get();
        } else if (camera == Cameras.FRONT_RIGHT_CAMERA.getIndex()) {
            this.hostname = photonVisionFrontRightHostname.get();
        } else {
            log.warn("Encountered an unrecognized camera value. Aborting to avoid unintended drive behavior.");
            return false;
        }

        if(this.leftOffsetPixelsSubscriber == null){
            this.leftOffsetPixelsSubscriber = new CachedSubscriber(hostname + "." + tableLeftDistance, client,5);
        }

        if(this.rightOffsetPixelsSubscriber == null){
            this.rightOffsetPixelsSubscriber = new CachedSubscriber(hostname + "." + tableRightDistance, client,5);
        }

        if(this.hresSubscriber == null){
            this.hresSubscriber = new CachedSubscriber(hostname + "." + tableHres, client,2);
        }

        if(this.vresSubscriber == null){
            this.vresSubscriber = new CachedSubscriber(hostname + "." + tableVres, client,2);
        }
        initalized = true;
        return true;
    }

    private boolean isDriveStopped(){
        return drive.getActiveSwerveModuleSubsystem().getCurrentState().equals(new SwerveModuleState(0, Rotation2d.kZero));
    }


    public boolean executeAlignment() {
        if(!this.initalized){
            log.error("AlignWithCreeperCommand initialization failed.");
            return false;
        }

        if (forceStop) {
            drive.stop();
            return false;
        }


        Integer leftDistance = this.leftOffsetPixelsSubscriber.getAsInteger(null);
        Integer rightDistance = this.rightOffsetPixelsSubscriber.getAsInteger(null);

        this.currentCamHres = this.hresSubscriber.getAsInteger(this.tunedWidth);
        this.currentCamVres = this.hresSubscriber.getAsInteger(this.tunedHeight);

        if (leftDistance == null || rightDistance == null) {
            // since subscriber is not updating fast enough, sometimes we get nulls
            aKitLog.record("Subscriber updated?", false);
            log.warn("Vision offsets are returning null! Is alignment up?");
            return false;
        }
        else{
            aKitLog.record("Subscriber updated?", true);
        }

        if (leftDistance == -1 && rightDistance == -1) {
            log.warn("No valid left or right distance measurements received from the camera.");
            forceStop = true;
            return false;
        }

        if(leftDistance == -1 || rightDistance == -1){
            this.isCenteredConfidently = false;
        }
        else{
            double resAdjustedDiff = normalizeWidth(Math.abs(leftDistance - rightDistance));
            this.isCenteredConfidently = resAdjustedDiff < this.errorThresholdPixels.get();
        }

        aKitLog.record("ConfidentlyCentered", this.isCenteredConfidently);
        aKitLog.record("LeftDistancePixelsAdjusted", leftDistance);
        aKitLog.record("RightDistancePixelsAdjusted", rightDistance);

        double error;
        if (isCenteredConfidently) {
            // Alignment is achieved; stop any drive movement.
            log.info("Vision is saying we are centered confidently!");
            // If the error is within the acceptable range, act as a "deadband"
            error = 0;
        } else {
            // Determine which side the misalignment error should be taken from.
            boolean offToTheLeft;

            if (leftDistance == -1) {
                offToTheLeft = false; // we don't see left edge, means right side is "too" visible e.g. off to the right
            } else if (rightDistance == -1) {
                offToTheLeft = true; // we don't see right edge, means left side is "too" visible e.g. off to the left
            } else {
                // Choose the pixel error value from the side indicating misalignment.
                offToTheLeft = leftDistance < rightDistance; // we see both edges, so the one that is "closer to the center" is the one we are off by
            }

            aKitLog.record("IsOffToTheLeft", offToTheLeft);

            // Calculate the error as a function of the left and right distance from the center.
            error = costFunc(leftDistance, rightDistance);

            // add sign
            // if we are off to the right (!offtotheleft) and we want to minimize our error in the right direction,
            // we need to invert
            if (!offToTheLeft) {
                error = -error;
            }
        }

        aKitLog.record("creeper error from cost function: ",error);

        // power.
        double drivePower = driveGain.get() * pidManager.calculate(0, error);
        aKitLog.record("Creeper Drive Power", drivePower);


        double push = Math.max(Math.min(this.pushForce.get(),1),0);  // clip to range 0-1

        XYPair pair = new XYPair(push, drivePower);

        this.drive.drive(pair, 0.0, true);

        return true;
    }

    // scaled cost function
    private double costFunc(int leftErrPX, int rightErrPX){
        if(leftErrPX == -1 || rightErrPX == -1){
            // much too off aligned
            return maxError.get();
        }
        // abs error
        double err = normalizeWidth(Math.abs(leftErrPX-rightErrPX));

//       /**Linear Error**/
        double errFunc = err*errorSlope.get();


        return Math.max(-maxError.get(), Math.min(errFunc, maxError.get())); // clip
    }

    public boolean isFinished(boolean waitUntilStop) {
        boolean isRegularFinish = this.isDriveStopped() && this.isCenteredConfidently;
        if (waitUntilStop) {
            // us being centered is only valuable if we arent currently moving
            return isRegularFinish;
        }
        else{
            // if we want to immediately stop
            return forceStop || isRegularFinish;
        }
    }


    public Cameras getCamera() {
        //return camera;
        return null;
    }

    public void setCamera(Cameras camera) {
        //this.camera = camera;
    }

    public void setCamera(int camera) {
        this.camera = camera;
    }

    private double normalizeWidth(double width){
        return width * this.currentCamHres / this.tunedWidth;
    }

    private double normalizeHeight(double height){
        return height * this.currentCamVres / this.tunedHeight;
    }

    // This will probably be enough for generally aligning with the other calculator
    public boolean getIsConfidentlyCentered() {
        return  isCenteredConfidently;
    }

    private static boolean isOffToTheLeft(int leftDistance, int rightDistance) {
        boolean offToTheLeft;

        if (leftDistance == -1) {
            offToTheLeft = false; // we don't see left edge, means right side is "too" visible e.g. off to the right
        } else if (rightDistance == -1) {
            offToTheLeft = true; // we don't see right edge, means left side is "too" visible e.g. off to the left
        } else {
            // Choose the pixel error value from the side indicating misalignment.
            offToTheLeft = leftDistance < rightDistance; // we see both edges, so the one that is "closer to the center" is the one we are off by
        }
        return offToTheLeft;
    }

    /**
     * NOTES: this could return 0 if leftdist or rightdist == null
     * that will make movement clunky with occasional 0s
     * @return a suggested power 0-1 relative to robot
     */
    public CreeperAlignmentSuggestion getSuggestedSidewaysAlignmentPower() {
        if (!initalized || forceStop) {
            return new CreeperAlignmentSuggestion(0);
        }

        Integer leftDistance = this.leftOffsetPixelsSubscriber.getAsInteger(null);
        Integer rightDistance = this.rightOffsetPixelsSubscriber.getAsInteger(null);

        this.currentCamHres = this.hresSubscriber.getAsInteger(this.tunedWidth);
        this.currentCamVres = this.hresSubscriber.getAsInteger(this.tunedHeight);

        // Occasionally we get nulls since the subscriber may not update quick enough
        if (leftDistance == null || rightDistance == null) {
            aKitLog.record("SubscriberUpdated", false); // This logging should be removed later for performance
            return new CreeperAlignmentSuggestion();
        } else {
            aKitLog.record("SubscriberUpdated", true);
        }

        aKitLog.record("LeftDistance", leftDistance);
        aKitLog.record("RightDistance", rightDistance);

        // Invalid left/right distance measurements
        if (leftDistance == -1 && rightDistance == -1) {
            return new CreeperAlignmentSuggestion(0);
        }

        if (leftDistance == -1 || rightDistance == -1) {
            isCenteredConfidently = false;
        } else {
            double resAdjustedDiff = normalizeWidth(Math.abs(leftDistance - rightDistance));
            isCenteredConfidently = resAdjustedDiff < this.errorThresholdPixels.get();
        }


        aKitLog.record("ConfidentlyCentered", isCenteredConfidently);

        double error = 0;
        if (!isCenteredConfidently) {
            // Determine which side the misalignment error should be taken from.
            boolean offToTheLeft = isOffToTheLeft(leftDistance, rightDistance);

            aKitLog.record("IsOffToTheLeft", offToTheLeft);

            // Calculate the error as a function of the left and right distance from the center.
            error = costFunc(leftDistance, rightDistance);

            // Need to invert error if right side
            if (!offToTheLeft) {
                error = -error;
            }
        }

        aKitLog.record("CreeperError", error);

        // We'll return a drive power that is ROBOT RELATIVE
        return new CreeperAlignmentSuggestion(driveGain.get() * pidManager.calculate(0, error));
    }
}