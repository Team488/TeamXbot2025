package competition.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.geometry.struct.Rotation2dStruct;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.JClient.XTablesClientManager;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class WPIXTablesClient {
    private final XTablesClientManager xTablesClient;
    private final Pose2dStruct pose2dStruct = new Pose2dStruct();
    private final Rotation2dStruct rotation2dStruct = new Rotation2dStruct();

    public WPIXTablesClient(XTablesClientManager xClient) {
        this.xTablesClient = xClient;
    }

    public void putPose2d(String table, Pose2d pose) {
        putPose2d("", table, pose);
    }

    public void putPose2d(String prefix, String table, Pose2d pose) {
        ByteBuffer bb = ByteBuffer.allocate(pose2dStruct.getSize());
        bb.order(ByteOrder.LITTLE_ENDIAN);
        pose2dStruct.pack(bb, pose);
        byte[] barr = new byte[bb.capacity()];
        bb.rewind();
        bb.get(barr);
        XTablesClient client = this.xTablesClient.getOrNull();
        if (client != null) client.putBytes(String.format("%s.%s", prefix, table), barr);
    }


}
