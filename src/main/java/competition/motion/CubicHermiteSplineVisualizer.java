package competition.motion;

import edu.wpi.first.math.geometry.Translation2d;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;

public class CubicHermiteSplineVisualizer extends JFrame {

    private static final double CANVAS_SIZE = 800;
    private static final double SCALE = 100;
    private static final double OFFSET = CANVAS_SIZE / 2;

    private CubicHermiteSpline spline;
    private double currentT = 0.0;
    private Timer animationTimer;
    private boolean isAnimating = false;
    private static final double ANIMATION_SPEED = 0.005; // Increment per timer tick

    private DrawPanel drawPanel;
    private JButton startButton;
    private JButton stopButton;
    private JButton restartButton;

    public CubicHermiteSplineVisualizer() {
        setTitle("Cubic Hermite Spline Visualizer");
        setSize((int) CANVAS_SIZE, (int) (CANVAS_SIZE + 50));  // Extra space for buttons
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        // Create a sample spline
        spline = new CubicHermiteSpline();
        spline.setStartPoint(new Translation2d( -3, 1));
        spline.setEndPoint(new Translation2d(3, -3));
        spline.setStartControlVector(6, 0);
        spline.setEndControlVector(6, +Math.PI/2);

        // Create drawing panel
        drawPanel = new DrawPanel();
        add(drawPanel, BorderLayout.CENTER);

        // Create control panel
        JPanel controlPanel = new JPanel();
        startButton = new JButton("Start");
        stopButton = new JButton("Stop");
        restartButton = new JButton("Restart");

        controlPanel.add(startButton);
        controlPanel.add(stopButton);
        controlPanel.add(restartButton);
        add(controlPanel, BorderLayout.SOUTH);

        // Set up the animation timer
        animationTimer = new Timer(16, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (currentT < 1.0) {
                    currentT += ANIMATION_SPEED;
                    if (currentT > 1.0) currentT = 1.0;
                    drawPanel.repaint();
                } else {
                    animationTimer.stop();
                    isAnimating = false;
                }
            }
        });

        // Configure button actions
        startButton.addActionListener(e -> {
            if (!isAnimating) {
                isAnimating = true;
                animationTimer.start();
            }
        });

        stopButton.addActionListener(e -> {
            if (isAnimating) {
                animationTimer.stop();
                isAnimating = false;
            }
        });

        restartButton.addActionListener(e -> {
            currentT = 0.0;
            drawPanel.repaint();
            if (isAnimating) {
                animationTimer.stop();
                animationTimer.start();
            }
        });
    }

    private class DrawPanel extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2d = (Graphics2D) g;
            g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            draw(g2d);
        }

        private void draw(Graphics2D g2d) {
            // Draw coordinate axes
            g2d.setColor(Color.LIGHT_GRAY);
            g2d.drawLine(0, (int) OFFSET, (int) CANVAS_SIZE, (int) OFFSET);  // X-axis
            g2d.drawLine((int) OFFSET, 0, (int) OFFSET, (int) CANVAS_SIZE);  // Y-axis

            // Draw the spline curve
            g2d.setColor(Color.BLUE);
            g2d.setStroke(new BasicStroke(2.0f));

            double prevX = spline.evaluateX(0);
            double prevY = spline.evaluateY(0);

            for (double t = 0.01; t <= 1.0; t += 0.01) {
                double x = spline.evaluateX(t);
                double y = spline.evaluateY(t);

                g2d.drawLine(
                    (int) (OFFSET + prevX * SCALE),
                    (int) (OFFSET - prevY * SCALE),
                    (int) (OFFSET + x * SCALE),
                    (int) (OFFSET - y * SCALE)
                );

                prevX = x;
                prevY = y;
            }

            // Draw start and end points
            int startPointSize = 8;
            g2d.setColor(Color.GREEN);
            g2d.fillOval(
                (int) (OFFSET + spline.getStartX() * SCALE - startPointSize/2),
                (int) (OFFSET - spline.getStartY() * SCALE - startPointSize/2),
                startPointSize, startPointSize
            );

            g2d.setColor(Color.MAGENTA);
            g2d.fillOval(
                (int) (OFFSET + spline.getEndX() * SCALE - startPointSize/2),
                (int) (OFFSET - spline.getEndY() * SCALE - startPointSize/2),
                startPointSize, startPointSize
            );

            // Draw control vectors as arrows
            drawArrow(g2d,
                spline.getStartX(), spline.getStartY(),
                spline.getStartX() + spline.getStartControlVectorX(),
                spline.getStartY() + spline.getStartControlVectorY(),
                Color.ORANGE
            );

            drawArrow(g2d,
                spline.getEndX(), spline.getEndY(),
                spline.getEndX() + spline.getEndControlVectorX(),
                spline.getEndY() + spline.getEndControlVectorY(),
                Color.CYAN
            );

            // Draw current position if animating or paused
            if (currentT > 0) {
                double currentX = spline.evaluateX(currentT);
                double currentY = spline.evaluateY(currentT);

                g2d.setColor(Color.RED);
                int currentPointSize = 12;
                g2d.fillOval(
                    (int) (OFFSET + currentX * SCALE - currentPointSize/2),
                    (int) (OFFSET - currentY * SCALE - currentPointSize/2),
                    currentPointSize, currentPointSize
                );
            }

            // Draw time information
            g2d.setColor(Color.BLACK);
            g2d.drawString(String.format("t = %.2f", currentT), 20, 20);
            g2d.drawString(String.format("Arc Length = %.2f", spline.estimateArcLength()), 20, 40);

            // Calculate and display velocity
            if (currentT > 0) {
                // Get current derivatives (velocity components)
                double dx = spline.derivativeX(currentT);
                double dy = spline.derivativeY(currentT);

                // Calculate velocity magnitude
                double velocity = Math.sqrt(dx * dx + dy * dy);

                // Display velocity
                g2d.drawString(String.format("Velocity = %.2f", velocity), 20, 60);
            }
        }

        private void drawArrow(Graphics2D g2d, double x1, double y1, double x2, double y2, Color color) {
            g2d.setColor(color);

            // Convert to screen coordinates
            double sx1 = OFFSET + x1 * SCALE;
            double sy1 = OFFSET - y1 * SCALE;
            double sx2 = OFFSET + x2 * SCALE;
            double sy2 = OFFSET - y2 * SCALE;

            // Draw main line
            g2d.drawLine((int)sx1, (int)sy1, (int)sx2, (int)sy2);

            // Calculate arrowhead
            double arrowSize = 10;
            double dx = sx2 - sx1;
            double dy = sy2 - sy1;
            double angle = Math.atan2(dy, dx);

            AffineTransform tx = new AffineTransform();
            tx.setToIdentity();
            tx.translate(sx2, sy2);
            tx.rotate(angle);

            // Create arrowhead shape
            int[] arrowX = {0, -10, -10};
            int[] arrowY = {0, -5, 5};

            g2d.setTransform(tx);
            g2d.fillPolygon(arrowX, arrowY, 3);

            // Reset transform
            g2d.setTransform(new AffineTransform());
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            CubicHermiteSplineVisualizer visualizer = new CubicHermiteSplineVisualizer();
            visualizer.setVisible(true);
        });
    }
}