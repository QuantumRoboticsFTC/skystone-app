package eu.qrobotics.skystone.teamcode.opmode.cv;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;

import java.util.Arrays;

import eu.qrobotics.skystone.teamcode.cv.trackers.FixedStoneTracker;
import eu.qrobotics.skystone.teamcode.cv.trackers.StoneType;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.AutoTrajectoryGenerator.SkystonePattern;
import eu.qrobotics.skystone.teamcode.util.StickyGamepad;

@Autonomous
@Config
public class FixedStoneVision extends LinearOpMode {

    public static int LEFT_STONE_UP_X = 70;
    public static int LEFT_STONE_UP_Y = 40;
    public static int LEFT_STONE_DOWN_X = 250;
    public static int LEFT_STONE_DOWN_Y = 150;
    public static int CENTER_STONE_UP_X = 200;
    public static int CENTER_STONE_UP_Y = 40;
    public static int CENTER_STONE_DOWN_X = 420;
    public static int CENTER_STONE_DOWN_Y = 150;
    public static int RIGHT_STONE_UP_X = 420;
    public static int RIGHT_STONE_UP_Y = 40;
    public static int RIGHT_STONE_DOWN_X = 600;
    public static int RIGHT_STONE_DOWN_Y = 150;

    private OpenCvCamera webcam;
    private OpenCvTrackerApiPipeline trackerApiPipeline;
    private FixedStoneTracker leftStone, centerStone, rightStone;
    private StoneType stoneType;
    private StickyGamepad stickyGamepad1;
    private SkystonePattern skystonePattern;

    @Override
    public void runOpMode() {
        stickyGamepad1 = new StickyGamepad(gamepad1);

        leftStone = new FixedStoneTracker(
                new Point(LEFT_STONE_UP_X, LEFT_STONE_UP_Y),
                new Point(LEFT_STONE_DOWN_X, LEFT_STONE_DOWN_Y)
        );
        centerStone = new FixedStoneTracker(
                new Point(CENTER_STONE_UP_X, CENTER_STONE_UP_Y),
                new Point(CENTER_STONE_DOWN_X, CENTER_STONE_DOWN_Y)
        );
        rightStone = new FixedStoneTracker(
                new Point(RIGHT_STONE_UP_X, RIGHT_STONE_UP_Y),
                new Point(RIGHT_STONE_DOWN_X, RIGHT_STONE_DOWN_Y)
        );

        trackerApiPipeline = new OpenCvTrackerApiPipeline();
        trackerApiPipeline.addTracker(leftStone);
        trackerApiPipeline.addTracker(centerStone);
        trackerApiPipeline.addTracker(rightStone);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(trackerApiPipeline);
        webcam.showFpsMeterOnViewport(true);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        while (!opModeIsActive() && !isStopRequested()) {
            stickyGamepad1.update();

            if (stickyGamepad1.a)
                webcam.pauseViewport();
            if (stickyGamepad1.b)
                webcam.resumeViewport();

            double[] counts = {average(leftStone.getCount()),
                    average(centerStone.getCount()),
                    average(rightStone.getCount())};

            int maxIdx = 0;
            double max = 0;
            for (int i = 0; i < counts.length; i++) {
                if (counts[i] > max) {
                    max = counts[i];
                    maxIdx = i;
                }
            }

            if (maxIdx == 0) {
                skystonePattern = SkystonePattern.LEFT;
            } else if (maxIdx == 1) {
                skystonePattern = SkystonePattern.MIDDLE;
            } else {
                skystonePattern = SkystonePattern.RIGHT;
            }

            telemetry.addData("Stone Type Left", leftStone.getCount());
            telemetry.addData("Stone Type Center", centerStone.getCount());
            telemetry.addData("Stone Type Right", rightStone.getCount());
            telemetry.addData("Skystone Pattern", skystonePattern);
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            sleep(100);
        }
    }

    private double average(Scalar s) {
        if (s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}
