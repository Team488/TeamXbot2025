package competition.motion;

import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Meters;

@Singleton
public class CommonRouteGenerator {

    final ReefCoordinateGenerator reefCoordinateGenerator;
    final PoseSubsystem pose;

    @Inject
    public CommonRouteGenerator(ReefCoordinateGenerator reefCoordinateGenerator, PoseSubsystem pose) {
        this.reefCoordinateGenerator = reefCoordinateGenerator;
        this.pose = pose;
    }

    // We're mostly interested in routes from the reef to the loading station and vice versa.
    // For going to the reef, we'll aim to be pointed at the reef with some small final velocity
    // relatively close so vision can do the final approach.
    // For going to the coral station, we'll just go straight there with a ~1 foot offset and modest velocity
    // and rely on a final shove for alignment

    Translation2d pointJustLeftOfReef = new Translation2d(4.5, 5.7);
    Translation2d pointJustAboveReef = new Translation2d(6.2, 4);

    public ArrayList<CubicHermiteSpline> getRouteFromReefToLoadingStation(
            Landmarks.ReefFace startingFace,
            Landmarks.CoralStation endingStation) {

        // All routes are tuned for blue alliance going to left coral station. If we want to go to the right coral station,
        // we mirror our starting face and get a route to left coral station, then mirror the whole route to get one that
        // would go to the right coral station.
        // Then, if we are on Red alliance, we field rotate the entire spline set.

        var currentPose = pose.getCurrentPose2d();

        if (endingStation == Landmarks.CoralStation.RIGHT) {
            startingFace = Landmarks.mirrorReefFaceLeftToRight(startingFace);
            // Pretend the robot is on the other side as well.
            currentPose = new Pose2d(
                    new Translation2d(currentPose.getX(), PoseSubsystem.mirrorYCoordinateAcrossMidfield(currentPose.getY())),
                            currentPose.getRotation().times(-1));
        }

        var splines = new ArrayList<CubicHermiteSpline>();
        var loadingStationEndPose = getLeftLoadingStationEndPose();

        var phase1 = new CubicHermiteSplineParameters();
        var phase2 = new CubicHermiteSplineParameters();

        switch (startingFace) {
            case FAR_LEFT:
            case CLOSE_LEFT:
            case CLOSE:
            case CLOSE_RIGHT:
                // All of these work with the basic "get away from face then go straight to station"
                phase1 = new CubicHermiteSplineParameters(
                        currentPose.getTranslation(),
                        loadingStationEndPose.getTranslation(),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.getReefFacePose(startingFace)).getRotation()
                                .plus(new Rotation2d(Math.PI))),
                        new Translation2d(5, loadingStationEndPose.getRotation()
                                .plus(new Rotation2d(Math.PI)))
                );
                splines.add(new CubicHermiteSpline(phase1));
                break;
            case FAR:
                // This one needs a little more work to get around the reef
                phase1 = new CubicHermiteSplineParameters(
                        currentPose.getTranslation(),
                        PoseSubsystem.convertBlueToRedIfNeeded(pointJustLeftOfReef),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(Rotation2d.fromDegrees(45))),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(Rotation2d.fromDegrees(180)))
                );
                phase2 = new CubicHermiteSplineParameters(
                        phase1.endPoint(),
                        loadingStationEndPose.getTranslation(),
                        phase1.endControlVector(),
                        new Translation2d(5, loadingStationEndPose.getRotation()
                                .plus(new Rotation2d(Math.PI)))
                );

                splines.add(new CubicHermiteSpline(phase1));
                splines.add(new CubicHermiteSpline(phase2));
                break;
            case FAR_RIGHT:
                phase1 = new CubicHermiteSplineParameters(
                        currentPose.getTranslation(),
                        PoseSubsystem.convertBlueToRedIfNeeded(pointJustAboveReef),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(Rotation2d.fromDegrees(-60))),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(Rotation2d.fromDegrees(90)))
                );
                phase2 = new CubicHermiteSplineParameters(
                        phase1.endPoint(),
                        PoseSubsystem.convertBlueToRedIfNeeded(pointJustLeftOfReef),
                        phase1.endControlVector(),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(Rotation2d.fromDegrees(180)))
                );
                var phase3 = new CubicHermiteSplineParameters(
                        phase2.endPoint(),
                        loadingStationEndPose.getTranslation(),
                        phase2.endControlVector(),
                        new Translation2d(5, loadingStationEndPose.getRotation()
                                .plus(new Rotation2d(Math.PI)))
                );

                splines.add(new CubicHermiteSpline(phase1));
                splines.add(new CubicHermiteSpline(phase2));
                splines.add(new CubicHermiteSpline(phase3));
                break;
            default:
        }


        if (endingStation == Landmarks.CoralStation.RIGHT) {
            splines = CubicHermiteSpline.mirrorSplineLeftToRight(splines);
        }

        return splines;
    }

    public ArrayList<CubicHermiteSpline> getRouteFromLoadingStationToReef(
            Landmarks.CoralStation startingStation,
            Landmarks.ReefFace endingFace, Landmarks.Branch endingBranch) {

        var currentPose = pose.getCurrentPose2d();
        var startingControlVector = Landmarks.BlueLeftCoralStationMid.getRotation();

        if (startingStation == Landmarks.CoralStation.RIGHT) {
            endingFace = Landmarks.mirrorReefFaceLeftToRight(endingFace);
            if (endingBranch == Landmarks.Branch.A) {
                endingBranch = Landmarks.Branch.B;
            } else {
                endingBranch = Landmarks.Branch.A;
            }
            //startingControlVector = startingControlVector.times(-1);
            // Pretend the robot is on the other side as well.
            currentPose = new Pose2d(
                    new Translation2d(currentPose.getX(), PoseSubsystem.mirrorYCoordinateAcrossMidfield(currentPose.getY())),
                    currentPose.getRotation().times(-1));
        }

        var splines = new ArrayList<CubicHermiteSpline>();
        var reefEndPose = reefCoordinateGenerator.getPoseRelativeToReefFaceAndBranch(
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue),
                endingFace,
                endingBranch,
                Meters.of(1.25),
                Meters.zero()
        );

        var phase1 = new CubicHermiteSplineParameters();
        var phase2 = new CubicHermiteSplineParameters();

        switch (endingFace) {
            case FAR_LEFT:
            case CLOSE_LEFT:
            case CLOSE:
            case CLOSE_RIGHT:
                phase1 = new CubicHermiteSplineParameters(
                        currentPose.getTranslation(),
                        reefEndPose.getTranslation(),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(startingControlVector)),
                        new Translation2d(5, reefEndPose.getRotation())
                );
                splines.add(new CubicHermiteSpline(phase1));
                break;
            case FAR:
                phase1 = new CubicHermiteSplineParameters(
                        currentPose.getTranslation(),
                        PoseSubsystem.convertBlueToRedIfNeeded(pointJustLeftOfReef),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(startingControlVector)),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(Rotation2d.fromDegrees(0)))
                );
                phase2 = new CubicHermiteSplineParameters(
                        phase1.endPoint(),
                        reefEndPose.getTranslation(),
                        phase1.endControlVector(),
                        new Translation2d(5, reefEndPose.getRotation())
                );
                splines.add(new CubicHermiteSpline(phase1));
                splines.add(new CubicHermiteSpline(phase2));
                break;
            case FAR_RIGHT:
                phase1 = new CubicHermiteSplineParameters(
                        currentPose.getTranslation(),
                        PoseSubsystem.convertBlueToRedIfNeeded(pointJustLeftOfReef),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(startingControlVector)),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(Rotation2d.fromDegrees(0)))
                );
                phase2 = new CubicHermiteSplineParameters(
                        phase1.endPoint(),
                        PoseSubsystem.convertBlueToRedIfNeeded(pointJustAboveReef),
                        phase1.endControlVector(),
                        new Translation2d(5, PoseSubsystem.convertBlueToRedIfNeeded(Rotation2d.fromDegrees(-90)))
                );
                var phase3 = new CubicHermiteSplineParameters(
                        phase2.endPoint(),
                        reefEndPose.getTranslation(),
                        phase2.endControlVector(),
                        new Translation2d(5, reefEndPose.getRotation())
                );
                splines.add(new CubicHermiteSpline(phase1));
                splines.add(new CubicHermiteSpline(phase2));
                splines.add(new CubicHermiteSpline(phase3));
                break;
            default:
                break;
        }

        if (startingStation == Landmarks.CoralStation.RIGHT) {
            splines = CubicHermiteSpline.mirrorSplineLeftToRight(splines);
        }

        return splines;
    }

    private Pose2d getLeftLoadingStationEndPose() {
        var coralStationPose = PoseSubsystem.convertBlueToRedIfNeeded(
                Landmarks.getCoralStationSectionPose(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        // From the pose, project a point in front of it
        var vectorToInterstitialPoint = new Translation2d(0.5, coralStationPose.getRotation());
        return new Pose2d(
                coralStationPose.getTranslation().plus(vectorToInterstitialPoint),
                coralStationPose.getRotation());
    }
}
