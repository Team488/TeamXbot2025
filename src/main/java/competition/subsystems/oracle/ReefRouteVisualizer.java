package competition.subsystems.oracle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.trajectory.XbotSwervePoint;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.List;

public class ReefRouteVisualizer extends JFrame {

    private static final double CANVAS_SIZE = 800;
    private static final double SCALE = 100;
    private static final double OFFSET = CANVAS_SIZE / 2;

    private ReefRoutingCircle routingCircle;
    private Pose2d startPose;
    private Pose2d endPose;

    public ReefRouteVisualizer() {
        setTitle("Reef Route Visualizer");
        setSize((int) CANVAS_SIZE, (int) CANVAS_SIZE);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        // Example data
        Translation2d center = new Translation2d(0, 0);
        double radius = 2.0;
        routingCircle = new ReefRoutingCircle(center, radius);
        startPose = new Pose2d(new Translation2d(-4, 0.05), new edu.wpi.first.math.geometry.Rotation2d());
        endPose = new Pose2d(new Translation2d(routingCircle.getInnerCollisionCircleRadius()+ 0.05, 0), new edu.wpi.first.math.geometry.Rotation2d());

        add(new DrawPanel());
    }

    private class DrawPanel extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            draw(g);
        }

        private void draw(Graphics g) {
            Graphics2D g2d = (Graphics2D) g;
            g2d.clearRect(0, 0, getWidth(), getHeight());

            // Draw x and y axes
            g2d.setColor(Color.BLACK);
            g2d.drawLine((int) OFFSET, 0, (int) OFFSET, (int) CANVAS_SIZE);
            g2d.drawLine(0, (int) OFFSET, (int) CANVAS_SIZE, (int) OFFSET);

            // Draw outer routing circle
            g2d.setColor(Color.BLUE);
            g2d.drawOval(
                    (int) (OFFSET + (routingCircle.getCenter().getX() - routingCircle.getRadius()) * SCALE),
                    (int) (OFFSET - (routingCircle.getCenter().getY() + routingCircle.getRadius()) * SCALE),
                    (int) (routingCircle.getRadius() * 2 * SCALE),
                    (int) (routingCircle.getRadius() * 2 * SCALE)
            );

            // Draw inner collision circle
            g2d.setColor(Color.RED);
            g2d.drawOval(
                    (int) (OFFSET + (routingCircle.getCenter().getX() - routingCircle.getInnerCollisionCircleRadius()) * SCALE),
                    (int) (OFFSET - (routingCircle.getCenter().getY() + routingCircle.getInnerCollisionCircleRadius()) * SCALE),
                    (int) (routingCircle.getInnerCollisionCircleRadius() * 2 * SCALE),
                    (int) (routingCircle.getInnerCollisionCircleRadius() * 2 * SCALE)
            );

            // Generate and draw swerve points
            List<XbotSwervePoint> swervePoints = routingCircle.generateSwervePoints(startPose, endPose);
            g2d.setColor(Color.GREEN);
            for (int i = 0; i < swervePoints.size() - 1; i++) {
                Translation2d start = swervePoints.get(i).getTranslation2d();
                Translation2d end = swervePoints.get(i + 1).getTranslation2d();
                g2d.drawLine(
                        (int) (OFFSET + start.getX() * SCALE),
                        (int) (OFFSET - start.getY() * SCALE),
                        (int) (OFFSET + end.getX() * SCALE),
                        (int) (OFFSET - end.getY() * SCALE)
                );
            }

            // Draw swerve points
            g2d.setColor(Color.GREEN);
            for (XbotSwervePoint point : swervePoints) {
                Translation2d translation = point.getTranslation2d();
                g2d.fillOval(
                        (int) (OFFSET + translation.getX() * SCALE) - 2,
                        (int) (OFFSET - translation.getY() * SCALE) - 2,
                        4,
                        4
                );
            }

            // Draw start point as a large green circle
            g2d.setColor(Color.GREEN);
            Translation2d startTranslation = startPose.getTranslation();
            g2d.fillOval(
                    (int) (OFFSET + startTranslation.getX() * SCALE) - 8,
                    (int) (OFFSET - startTranslation.getY() * SCALE) - 8,
                    16,
                    16
            );

            // Draw end point as a large red circle
            g2d.setColor(Color.RED);
            Translation2d endTranslation = endPose.getTranslation();
            g2d.fillOval(
                    (int) (OFFSET + endTranslation.getX() * SCALE) - 8,
                    (int) (OFFSET - endTranslation.getY() * SCALE) - 8,
                    16,
                    16
            );
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            ReefRouteVisualizer visualizer = new ReefRouteVisualizer();
            visualizer.setVisible(true);
        });
    }
}