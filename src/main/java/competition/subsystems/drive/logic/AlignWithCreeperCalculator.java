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
    private final DoubleProperty errorThreshold;
    private final DoubleProperty pushForce;

    private CachedSubscriber leftOffsetPixelsSubscriber;
    private CachedSubscriber rightOffsetPixelsSubscriber;
    private CachedSubscriber hresSubscriber;
    private CachedSubscriber vresSubscriber;


    private String hostname;
    private Cameras camera;

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
        this.errorThreshold = pf.createPersistentProperty(
                "error Threshold [0-1]", 0.1);
        this.pushForce = pf.createPersistentProperty("Creeper push force", 0);
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
        if (camera.equals(Cameras.FRONT_LEFT_CAMERA)) {
            this.hostname = photonVisionFrontLeftHostname.get();
        } else if (camera.equals(Cameras.FRONT_RIGHT_CAMERA)) {
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

        if(forceStop){
//            log.info("FORCE STOP");
            drive.stop();
            return false;
        }

        // get network distances
        Integer leftDistance = this.leftOffsetPixelsSubscriber.getAsInteger(null);
        Integer rightDistance = this.rightOffsetPixelsSubscriber.getAsInteger(null);

        this.currentCamHres = this.hresSubscriber.getAsInteger(this.tunedWidth);
        this.currentCamVres = this.hresSubscriber.getAsInteger(this.tunedHeight);

        if (leftDistance == null || rightDistance == null) {
            // since subscriber is not updating fast enough, sometimes we get nulls
            aKitLog.record("Subcriber updated?", false);
            log.warn("Vision offsets are returning null! Is alignment up?");
            return false;
        }
        else{
            aKitLog.record("Subcriber updated?", true);
        }

        if (leftDistance == -1 && rightDistance == -1) {
            log.warn("No valid left or right distance measurements received from the camera.");
            forceStop = true;
            return false;
        }

        // find error
        double error;
        if(leftDistance == -1 || rightDistance == -1){
            error = 1;
        }
        else{
            error = costFunc(leftDistance, rightDistance);
        }

        boolean offToTheLeft = error < 0;


        aKitLog.record("creeper error",error);   
        aKitLog.record("Off to the left",offToTheLeft);   

        this.isCenteredConfidently = Math.abs(error) < this.errorThreshold.get();


        aKitLog.record("Is Centered confidently", this.isCenteredConfidently);
        aKitLog.record("Left Distance Pixels Adjusted", leftDistance);
        aKitLog.record("Right Distance Pixels Adjusted", rightDistance);

        if (!isCenteredConfidently) {
            double drivePower = driveGain.get() * pidManager.calculate(0, error);
            aKitLog.record("Creeper Drive Power", drivePower);


            double push = this.pushForce.get();

            XYPair pair = new XYPair(push, drivePower);

            this.drive.drive(pair, 0.0, true);
        }

        return true;

    }


    // scaled cost function
    private double costFunc(int leftSidePx, int rightSidePx){
        double screenCenter = this.currentCamHres/2;
        double midPix = (leftSidePx + rightSidePx) /2;

        double err = midPix-screenCenter;

        return err/screenCenter; // this will normalize to range [-1, 1]
    }

    public boolean isFinished(boolean waitUntilStop) {
        boolean isRegularFinish = this.isDriveStopped() && this.isCenteredConfidently;
        if(waitUntilStop){
            // us being centered is only valuable if we arent currently moving
            return isRegularFinish;
        }
        else{
            // if we want to immediately stop
            return forceStop || isRegularFinish;
        }
    }


    public Cameras getCamera() {
        return camera;
    }

    public void setCamera(Cameras camera) {
        this.camera = camera;
    }

}
