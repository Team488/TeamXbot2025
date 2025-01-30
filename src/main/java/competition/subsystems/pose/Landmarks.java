package competition.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Landmarks {

    // Auto starting positions
    public static Pose2d BlueLeftStartingLine = new Pose2d(7.56, 7.262, Rotation2d.fromDegrees(180));
    public static Pose2d BlueMidStartingLine = new Pose2d(7.56, 6.135, Rotation2d.fromDegrees(180));
    public static Pose2d BlueRightStartingLine = new Pose2d(7.56, 5.044, Rotation2d.fromDegrees(180));

    // Reef faces
    // Close Left Reef Face
    public static Pose2d BlueCloseLeftBranchA = new Pose2d(4.002, 5.174, Rotation2d.fromDegrees(-60));
    public static Pose2d BlueCloseLeftAlgae = new Pose2d(3.834 ,5.137, Rotation2d.fromDegrees(-60));
    public static Pose2d BlueCloseLeftBranchB = new Pose2d(3.717, 5.010, Rotation2d.fromDegrees(-60));

    // Close Reef Face
    public static Pose2d BlueCloseBranchA = new Pose2d(3.2385, 4.1699, Rotation2d.fromDegrees(0));
    public static Pose2d BlueCloseAlgae = new Pose2d(3.2385, 4.0259, Rotation2d.fromDegrees(0));
    public static Pose2d BlueCloseBranchB = new Pose2d(3.2385, 3.8412, Rotation2d.fromDegrees(0));

    // Close Right Reef Face
    public static Pose2d BlueCloseRightBranchA = new Pose2d(3.718, 3.042, Rotation2d.fromDegrees(60));
    public static Pose2d BlueCloseRightAlgae = new Pose2d(3.834, 2.915, Rotation2d.fromDegrees(60));
    public static Pose2d BlueCloseRightBranchB = new Pose2d(4.002, 2.877, Rotation2d.fromDegrees(60));

    // Far Right Reef Face
    public static Pose2d BlueFarRightBranchA = new Pose2d(4.948, 2.878, Rotation2d.fromDegrees(120));
    public static Pose2d BlueFarRightAlgae = new Pose2d(5.116, 2.915, Rotation2d.fromDegrees(120));
    public static Pose2d BlueFarRightBranchB = new Pose2d(5.233, 3.041, Rotation2d.fromDegrees(120));

    // Far Reef Face
    public static Pose2d BlueFarBranchA = new Pose2d(5.701, 3.841, Rotation2d.fromDegrees(180));
    public static Pose2d BlueFarAlgae = new Pose2d(5.701, 4.026, Rotation2d.fromDegrees(180));
    public static Pose2d BlueFarBranchB = new Pose2d(5.701, 4.170, Rotation2d.fromDegrees(180));

    // Far Left Reef Face
    public static Pose2d BlueFarLeftBranchA = new Pose2d(5.233, 5.009, Rotation2d.fromDegrees(-120));
    public static Pose2d BlueFarLeftAlgae = new Pose2d(5.116, 5.137, Rotation2d.fromDegrees(-120));
    public static Pose2d BlueFarLeftBranchB = new Pose2d(4.949, 5.175, Rotation2d.fromDegrees(-120));

    // Coral Stations
    // Left Coral Station
    public static Pose2d BlueLeftCoralStationClose = new Pose2d(0.765, 6.804, Rotation2d.fromDegrees(-54.012));
    public static Pose2d BlueLeftCoralStationMid = new Pose2d(1.093, 7.043, Rotation2d.fromDegrees(-54.012));
    public static Pose2d BlueLeftCoralStationFar = new Pose2d(1.442, 7.282, Rotation2d.fromDegrees(-54.012));

    // Right Coral Station
    public static Pose2d BlueRightCoralStationClose = new Pose2d(0.764, 1.247, Rotation2d.fromDegrees(54.012));
    public static Pose2d BlueRightCoralStationMid = new Pose2d(1.093, 1.009, Rotation2d.fromDegrees(54.012));
    public static Pose2d BlueRightCoralStationFar = new Pose2d(1.422, 0.770, Rotation2d.fromDegrees(54.012));



}