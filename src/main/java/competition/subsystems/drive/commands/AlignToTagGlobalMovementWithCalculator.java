package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DrivePowerCalculator;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AlignCameraToAprilTagCalculator;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class AlignToTagGlobalMovementWithCalculator extends BaseCommand {
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    DriveSubsystem drive;
    HeadingModule headingModule;
    ElectricalContract electricalContract;
    PoseSubsystem pose;

    private int targetAprilTagID;
    private int targetCameraID;

    AlignCameraToAprilTagCalculator calculator;

    @Inject
    public AlignToTagGlobalMovementWithCalculator(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
                                                  HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
                                                  ElectricalContract electricalContract) {
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.electricalContract = electricalContract;
        this.drive = drive;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.pose = pose;
        addRequirements(drive);
    }

    public void setAprilTagTarget(int targetAprilTagID) {
        this.targetAprilTagID = targetAprilTagID;
    }

    public void setCameraTarget(int targetCameraID) {
        this.targetCameraID = targetCameraID;
    }

    double headingAtCommandStart = 0;

    @Override
    public void initialize() {
        log.info("Initializing");
        drive.getPositionalPid().reset();
        headingAtCommandStart = pose.getCurrentPose2d().getRotation().getDegrees();
        calculator = new AlignCameraToAprilTagCalculator(aprilTagVisionSubsystem, electricalContract,
                drive.getPositionalPid(), targetAprilTagID, targetCameraID, 24, false);
    }

    @Override
    public void execute() {
        Translation2d driveValues = calculator.getXYPowersAlignToAprilTag(pose.getCurrentPose2d());
        double omega = headingModule.calculateHeadingPower(headingAtCommandStart);
        drive.fieldOrientedDrive(
                new XYPair(driveValues.getX(), driveValues.getY()),
                omega,
                pose.getCurrentHeading().getDegrees(),
                true);
    }

    @Override
    public boolean isFinished() {
        return drive.getPositionalPid().isOnTarget()
                && drive.getRotateToHeadingPid().isOnTarget()
                && calculator.getTagAcquisitionState() != AlignCameraToAprilTagCalculator.TagAcquisitionState.NeverSeen;
    }
}