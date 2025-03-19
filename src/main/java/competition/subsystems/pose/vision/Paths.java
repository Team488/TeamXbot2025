package competition.subsystems.pose.vision;

import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.kobe.xbot.Utilities.Entities.XTableValues;

import java.util.List;

import static edu.wpi.first.units.Units.Meters;

/**
 * The Paths class provides pre-defined Bézier curve paths for robot trajectories
 * used in the vision subsystem. All paths are defined for the left blue side.
 * It also provides utility methods for mirroring these paths to the right side
 * by reflecting control point Y coordinates over the field's Y axis midpoint.
 */
public class Paths {

    /**
     * Enum representing the field side.
     */
    public enum Side {
        LEFT,  // Represents the left blue side.
        RIGHT  // Represents the right blue side.
    }

    // Pre-defined Bezier curve paths for trajectories on the left blue side.
    public static XTableValues.BezierCurves blueCageOneStartingLineToFarLeftBranchA =
            XTableValues.BezierCurves.newBuilder()
                    .setOptions(XTableValues.TraversalOptions.newBuilder()
                            .setFinalRotationDegrees(-120.00)
                            .build())
                    .addCurves(0, XTableValues.BezierCurve.newBuilder()
                            .addAllControlPoints(List.of(
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(7.1217391304347837)
                                            .setY(7.2348101265822793)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(5.0869565217391308)
                                            .setY(5.1968354430379753)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(4.9852173913043485)
                                            .setY(5.1968354430379753)
                                            .build()
                            ))
                            .build())
                    .build();

    public static XTableValues.BezierCurves farLeftAToCoralStationBlue =
            XTableValues.BezierCurves.newBuilder()
                    .setOptions(XTableValues.TraversalOptions.newBuilder()
                            .setFinalRotationDegrees(-54)
                            .build())
                    .addCurves(0, XTableValues.BezierCurve.newBuilder()
                            .addAllControlPoints(List.of(
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(4.9852173913043485)
                                            .setY(5.1968354430379753)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(3.179347826086957)
                                            .setY(7.0055379746835449)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(1.1191304347826088)
                                            .setY(7.0055379746835449)
                                            .build()
                            ))
                            .build())
                    .build();

    public static XTableValues.BezierCurves blueLeftCoralStationToCloseLeftBranchA =
            XTableValues.BezierCurves.newBuilder()
                    .setOptions(XTableValues.TraversalOptions.newBuilder()
                            .setFinalRotationDegrees(-60.00)
                            .build())
                    .addCurves(0, XTableValues.BezierCurve.newBuilder()
                            .addAllControlPoints(List.of(
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(1.1191304347826088)
                                            .setY(7.0055379746835449)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(2.9250000000000003)
                                            .setY(5.1968354430379753)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(3.9678260869565221)
                                            .setY(5.1968354430379753)
                                            .build()
                            ))
                            .build())
                    .build();

    public static XTableValues.BezierCurves blueCloseLeftBranchBToLeftCoralStation =
            XTableValues.BezierCurves.newBuilder()
                    .setOptions(XTableValues.TraversalOptions.newBuilder()
                            .setFinalRotationDegrees(-54)
                            .build())
                    .addCurves(0, XTableValues.BezierCurve.newBuilder()
                            .addAllControlPoints(List.of(
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(3.68804347826087)
                                            .setY(5.0439873417721524)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(1.7295652173913045)
                                            .setY(7.0055379746835449)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(1.1191304347826088)
                                            .setY(7.0055379746835449)
                                            .build()
                            ))
                            .build())
                    .build();

    public static XTableValues.BezierCurves blueCloseLeftBranchAToLeftCoralStation =
            XTableValues.BezierCurves.newBuilder()
                    .setOptions(XTableValues.TraversalOptions.newBuilder()
                            .setFinalRotationDegrees(-54)
                            .build())
                    .addCurves(0, XTableValues.BezierCurve.newBuilder()
                            .addAllControlPoints(List.of(
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(3.9678260869565221)
                                            .setY(5.1968354430379753)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(2.1619565217391306)
                                            .setY(7.0055379746835449)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(1.1191304347826088)
                                            .setY(7.0055379746835449)
                                            .build()
                            ))
                            .build())
                    .build();

    public static XTableValues.BezierCurves blueLeftCoralStationToCloseBranchA =
            XTableValues.BezierCurves.newBuilder()
                    .setOptions(XTableValues.TraversalOptions.newBuilder()
                            .setFinalRotationDegrees(0)
                            .build())
                    .addCurves(0, XTableValues.BezierCurve.newBuilder()
                            .addAllControlPoints(List.of(
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(1.1191304347826088)
                                            .setY(7.0055379746835449)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(3.179347826086957)
                                            .setY(4.9420886075949371)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(3.179347826086957)
                                            .setY(4.1778481012658233)
                                            .build()
                            ))
                            .build())
                    .build();

    public static XTableValues.BezierCurves blueCloseBranchAToLeftCoralStation =
            XTableValues.BezierCurves.newBuilder()
                    .setOptions(XTableValues.TraversalOptions.newBuilder()
                            .setFinalRotationDegrees(-54)
                            .build())
                    .addCurves(0, XTableValues.BezierCurve.newBuilder()
                            .addAllControlPoints(List.of(
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(3.179347826086957)
                                            .setY(4.1778481012658233)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(3.179347826086957)
                                            .setY(4.9420886075949371)
                                            .build(),
                                    XTableValues.ControlPoint.newBuilder()
                                            .setX(1.1191304347826088)
                                            .setY(7.0055379746835449)
                                            .build()
                            ))
                            .build())
                    .build();

    /**
     * Mirrors the given Bezier curve path based on the specified field side.
     * <p>
     * All pre-defined paths are created for the left blue side. When the right side is requested,
     * this method mirrors each control point by reflecting its Y coordinate about the field's Y midpoint.
     * The field Y midpoint is calculated as half of the fieldYHeight. The mirroring operation for a control point is:
     * <br>
     * mirroredY = 2 * (fieldYMidpoint) - originalY
     * <br>
     * Additionally, the final rotation degrees from the traversal options are mirrored by multiplying by -1.
     * </p>
     *
     * @param side         the desired field side (LEFT or RIGHT)
     * @param originalPath the original Bezier curve path (from the left blue side)
     * @return the original path if LEFT is selected, or a new mirrored path if RIGHT is selected
     */
    public static XTableValues.BezierCurves mirrorPathLeftOrRight(Side side, XTableValues.Alliance alliance, XTableValues.BezierCurves originalPath) {
        // If the left side is selected, return the original path without modification.
        if (side == Side.LEFT && alliance.equals(XTableValues.Alliance.BLUE)) {
            return originalPath;
        }

        // Calculate the field Y midpoint.
        double fieldYMidpoint = PoseSubsystem.fieldYMidpointInMeters.in(Meters);
        double fieldXMidpoint = PoseSubsystem.fieldXMidpointInMeters.in(Meters);


        // Create a new builder for the mirrored path.
        XTableValues.BezierCurves.Builder mirroredPathBuilder = originalPath.toBuilder();
        // Clear existing curves to add mirrored curves.
        mirroredPathBuilder.clearCurves();

        // Iterate over each BezierCurve in the original path.
        for (XTableValues.BezierCurve curve : originalPath.getCurvesList()) {
            XTableValues.BezierCurve.Builder mirroredCurveBuilder = XTableValues.BezierCurve.newBuilder();

            // Iterate over each control point in the current curve.
            for (XTableValues.ControlPoint cp : curve.getControlPointsList()) {
                double originalY = cp.getY();
                double originalX = cp.getX();
                // Compute the mirrored Y coordinate by reflecting about the field's Y midpoint.
                double mirroredY = 2 * fieldYMidpoint - originalY;

                Translation2d mirroredPose = new Translation2d(originalX, mirroredY);
                if(alliance.equals(XTableValues.Alliance.RED)) {
                   mirroredPose = PoseSubsystem
                            .convertBlueToRed(new Translation2d(originalX, side.equals(Side.LEFT) ? originalY : mirroredY));
                }

                // Build a new control point with the mirrored Y coordinate.
                XTableValues.ControlPoint mirroredCp = cp.toBuilder()
                        .setY(mirroredPose.getY())
                        .setX(mirroredPose.getX())
                        .build();
                mirroredCurveBuilder.addControlPoints(mirroredCp);
            }
            // Add the mirrored curve to the new path.
            mirroredPathBuilder.addCurves(mirroredCurveBuilder.build());
        }

        // Mirror the final rotation degrees in the traversal options.
        if (originalPath.hasOptions()) {
            XTableValues.TraversalOptions originalOptions = originalPath.getOptions();
            double originalFinalRotation = originalOptions.getFinalRotationDegrees();
            if(side.equals(Side.RIGHT)) {
                originalFinalRotation = -originalFinalRotation + 180;
            }

            Rotation2d finalRotation = PoseSubsystem
                        .convertBlueToRed(Rotation2d.fromDegrees(originalFinalRotation));
            if(alliance.equals(XTableValues.Alliance.RED) && side.equals(Side.RIGHT)) {
               finalRotation = finalRotation.plus(Rotation2d.fromDegrees(180));
            }
            XTableValues.TraversalOptions mirroredOptions = originalOptions.toBuilder()
                    .setFinalRotationDegrees(finalRotation.getDegrees())
                    .build();
            mirroredPathBuilder.setOptions(mirroredOptions);
        }

        // Copy over alignment options if they exist.
        if (originalPath.hasAlignToReefAprilTagOptions()) {
            mirroredPathBuilder.setAlignToReefAprilTagOptions(originalPath.getAlignToReefAprilTagOptions());
        }

        return mirroredPathBuilder.build();
    }
}
