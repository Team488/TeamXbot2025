package competition.subsystems.drive.commands.vision_path;

import competition.commandgroups.vision_path.PathToFaceAndScoreCommandGroupFactory;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.kobe.xbot.JClient.XTablesClient;

import javax.inject.Inject;
import java.util.Set;

public class DriveToNearestOpenReef extends DeferredCommand{

    @Inject
    public DriveToNearestOpenReef(CoprocessorCommunicationSubsystem communicator, PathToFaceAndScoreCommandGroupFactory commandGroupFactory) {
        super(() -> {
            XTablesClient client = communicator.getXTablesManager().getOrNull();
            if (client == null) {
                return new InstantCommand(() -> System.out.println("Client is null, cannot generate command."));
            }

            try {
                int bestAT = client.getInteger("BESTOPENREEF_AT");
                int bestBranchIDX = client.getInteger("BESTOPENREEFBRANCH");

                System.out.println(String.format("Best april tag id: %d Best branch idx: %d", bestAT, bestBranchIDX));

                if (bestAT == -1 || bestBranchIDX == -1) {
                    return new InstantCommand(() -> System.out.println("No valid reef found, skipping command."));
                }

                // Get the reef face, branch, and height
                Landmarks.ReefFace face = Landmarks.getReefFaceFromTagId(bestAT);
                Landmarks.Branch branch = (bestBranchIDX % 2 == 0) ? Landmarks.Branch.A : Landmarks.Branch.B;
                Landmarks.CoralLevel height = getLevel(bestBranchIDX);

                // Return the created command group
                return commandGroupFactory.create(face, branch, height);

            } catch (IllegalArgumentException illegalArgs) {
                System.out.println("Exception occurred: " + illegalArgs.toString());
                return new InstantCommand(() -> System.out.println("Exception in DriveToNearestOpenReef"));
            }
        }, Set.of());
    }

    private static Landmarks.CoralLevel getLevel(int branchIdx) {
        int height = (branchIdx / 2) + 1;  // Calculate height

        switch (height) {  // Use height instead of getLevel
            case 1:
                return Landmarks.CoralLevel.ONE;
            case 2:
                return Landmarks.CoralLevel.TWO;
            case 3:
                return Landmarks.CoralLevel.THREE;
            case 4:
                return Landmarks.CoralLevel.FOUR;
            default:
                throw new IllegalArgumentException("Invalid branchIdx: " + branchIdx);
        }
    }
}
