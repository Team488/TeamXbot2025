package competition.subsystems.drive.logic;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.kobe.xbot.JClient.CachedSubscriber;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.JClient.XTablesClientManager;
import xbot.common.advantage.AKitLogger;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;


public class AlignWithCreeperLogger {
    private boolean initalized = false;

    private final Logger log;
    private final AKitLogger aKitLog;

    private final int tunedWidth = 960; // what resolution the pid tuning was done at

    private final int tunedHeight = 720; // what resolution the pid tuning was done at
    private final String tableLeftDistance = "verticalEdgeLeftDistancePx";

    private final String tableRightDistance = "verticalEdgeRightDistancePx";
    private final String tableHres = "alignmentCameraHres";
    private final String tableVres = "alignmentCameraVres";

    private CachedSubscriber leftOffsetPixelsSubscriber;

    private CachedSubscriber rightOffsetPixelsSubscriber;
    private CachedSubscriber hresSubscriber;
    private CachedSubscriber vresSubscriber;

    private XTablesClientManager xtablesManager;

    // set defaults for now

    private int currentCamHres = this.tunedWidth;
    private int currentCamVres = this.tunedHeight;
    private Boolean isCenteredConfidently = false;

    public AlignWithCreeperLogger(XTablesClientManager xTablesClientManager, AKitLogger aKitLogger) {
        this.log = LogManager.getLogger("AlignWithCreeperCommmand");
        this.aKitLog = aKitLogger;

        this.xtablesManager = xTablesClientManager;
    }


    public boolean initialize() {
        isCenteredConfidently = false;

        // try get xtables client
        XTablesClient client =
                this.xtablesManager.getOrNull();

        if (client == null) {
            return false;
        }

        if(this.leftOffsetPixelsSubscriber == null){
            this.leftOffsetPixelsSubscriber = new CachedSubscriber(tableLeftDistance, client,5);
        }

        if(this.rightOffsetPixelsSubscriber == null){
            this.rightOffsetPixelsSubscriber = new CachedSubscriber(tableRightDistance, client,5);
        }

        if(this.hresSubscriber == null){
            this.hresSubscriber = new CachedSubscriber(tableHres, client,2);
        }

        if(this.vresSubscriber == null){
            this.vresSubscriber = new CachedSubscriber(tableVres, client,2);
        }
        initalized = true;
        return true;
    }


    public void logAlignment(){
        if(!this.initalized){
            return;
        }

        // get network distances
        Integer leftDistance = this.leftOffsetPixelsSubscriber.getAsInteger(null);
        Integer rightDistance = this.rightOffsetPixelsSubscriber.getAsInteger(null);

        this.currentCamHres = this.hresSubscriber.getAsInteger(this.tunedWidth);
        this.currentCamVres = this.hresSubscriber.getAsInteger(this.tunedHeight);

        if (leftDistance == null || rightDistance == null) {
            // since subscriber is not updating fast enough, sometimes we get nulls
            aKitLog.record("Subcriber updated?", false);
            return;
        }
        else{
            aKitLog.record("Subcriber updated?", true);
        }

        aKitLog.record("Left Distance Pixels Adjusted", leftDistance);
        aKitLog.record("Right Distance Pixels Adjusted", rightDistance);


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

    }


    // scaled cost function

    private double costFunc(int leftSidePx, int rightSidePx){
        double screenCenter = this.currentCamHres/2;
        double midPix = (leftSidePx + rightSidePx) /2;

        double err = midPix-screenCenter;

        return err/screenCenter; // this will normalize to range [-1, 1]
    }

    public boolean isInitalized() {
        return initalized;
    }

}
