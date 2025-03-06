package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import dagger.assisted.AssistedFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import org.kobe.xbot.JClient.XTablesClient;
import xbot.common.advantage.AKitLogger;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.vision.AprilTagVisionIO;

import javax.inject.Inject;
import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class AlignWithCreeperCommand extends BaseCommand {

    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    final HeadingModule headingModule;
    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final AKitLogger akitLog;
    final ElectricalContract electricalContract;
    final ReefCoordinateGenerator reefCoordinateGenerator;
    final CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem;

    private final String tablelLeftDistance = "verticalEdgeLeftDistancePx";
    private final String tablelRightDistance = "verticalEdgeRightDistancePx";
    private final String tablelCenteredConfidently = "verticalAlignedConfidently";

    private int leftDistance;
    private int rightDistance;
    private boolean centeredConfidently = false;
    int targetAprilTagID;
    int targetCameraID;
    double initialHeading;
    Translation2d alignmentPointOffset;
    Rotation3d cameraRotation;

    private boolean isCameraBackwards = false;
    Translation2d targetLocationOnField = new Translation2d(0, 0);

    double lastKnownHorizontalErrorMeters = 0;
    double shoveStartTime = 0;
    private boolean requireExcellentAlignment = true;

    public static Translation2d generateAlignmentPointOffset(Distance robotCenterToOuterBumperX, CameraInfo cameraInfo,
                                                             Distance offset, boolean isCameraBackwards) {
        return new Translation2d(
                robotCenterToOuterBumperX.times(isCameraBackwards ? -1 : 1)
                        .minus(cameraInfo.position().getMeasureX())
                        .plus(offset.times(isCameraBackwards? -1 : 1)),
                Meters.zero()
        );
    }

    String prefix = "AlignWithCreeperCommand/";
    @Inject
    public AlignWithCreeperCommand(AprilTagVisionSubsystemExtended vision, DriveSubsystem drive, CoprocessorCommunicationSubsystem coprocessorCommunications,
                                           ElectricalContract electricalContract, PoseSubsystem pose,
                                           HeadingModule.HeadingModuleFactory headingModuleFactory, ReefCoordinateGenerator reefCoordinateGenerator,
                                           PropertyFactory pf) {
        this.aprilTagVisionSubsystem = vision;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.drive = drive;
        this.pose = pose;
        this.electricalContract = electricalContract;
        this.coprocessorCommunicationSubsystem = coprocessorCommunications;
        this.reefCoordinateGenerator = reefCoordinateGenerator;
        this.akitLog = new AKitLogger(prefix);

        pf.setPrefix(prefix);

    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute(){
        super.execute();
        XTablesClient client = this.coprocessorCommunicationSubsystem.tryGetXTablesClient();
        if(client == null){
            System.out.print("CoprocessorCommunicationSubsystem could not get XTablesClient\n");
            return;
        }

        try{
            this.leftDistance = client.getInteger(tablelLeftDistance);
            this.rightDistance = client.getInteger(tablelRightDistance);
            this.centeredConfidently = client.getBoolean(tablelCenteredConfidently);
            this.akitLog.record("leftDistance", this.leftDistance);
            this.akitLog.record("rightDistance", this.rightDistance);
            this.akitLog.record("isCenteredConfidently", this.centeredConfidently);
            if(this.centeredConfidently){
                return; // no work to do
            }
            boolean offToTheLeft;
            if(this.leftDistance == -1 && this.rightDistance == -1){
                System.out.print("Could not get either distance!");
                return;
            }
            else if(this.leftDistance == -1){
                offToTheLeft = false;
            }
            else if(this.rightDistance == -1){
                offToTheLeft = true;
            }
            else{
                offToTheLeft = this.leftDistance < this.rightDistance;
            }

            int moveAmount;
            int dir;
            if(offToTheLeft){
                moveAmount = this.leftDistance;
                dir = 1;
            }
            else{
                moveAmount = this.rightDistance;
                dir = -1;
            }
            float stepSizeCM;
            if(moveAmount > 50){
                stepSizeCM = 8;
            }
            else if(moveAmount > 25){
                stepSizeCM = 6;
            }
            else{
                stepSizeCM = 2;
            }

            float translationYMeters = (stepSizeCM * dir)/100;

            this.drive.fieldOrientedDrive(new XYPair(0,translationYMeters), 0,pose.getCurrentHeading().getRadians(),false);

        }
        catch (Exception e){
            System.out.print(e.getMessage());
        }




    }

    @Override
    public boolean isFinished(){
        return this.centeredConfidently;
    }

}
