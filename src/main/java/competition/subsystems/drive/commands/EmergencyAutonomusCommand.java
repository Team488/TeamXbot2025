package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Power;

import javax.inject.Inject;


public class EmergencyAutonomusCommand extends DriveToLocationWithPID {
    DriveSubsystem driveSubsystem;
    PoseSubsystem poseSubsystem;
    Translation2d locationStarting = new Translation2d(0,0);
    Translation2d locationTarget = new Translation2d(0, 1);

    double currentPostion;
    double oldPostion;

    @Inject
    public EmergencyAutonomusCommand(DriveSubsystem drive, PoseSubsystem pose) {
        super(drive, pose);

        setLocationTarget(locationTarget);



    }


}