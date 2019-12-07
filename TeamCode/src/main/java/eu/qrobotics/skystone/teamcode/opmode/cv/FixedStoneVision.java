package eu.qrobotics.skystone.teamcode.opmode.cv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;

import eu.qrobotics.skystone.teamcode.cv.trackers.FixedStoneTracker;
import eu.qrobotics.skystone.teamcode.util.StickyGamepad;

@Autonomous
public class FixedStoneVision extends LinearOpMode {

    private OpenCvCamera webcam;
    private OpenCvTrackerApiPipeline trackerApiPipeline;
    private FixedStoneTracker leftStone, centerStone, rightStone;
    private StickyGamepad stickyGamepad1;

    @Override
    public void runOpMode()
    {
        stickyGamepad1 = new StickyGamepad(gamepad1);

        leftStone = new FixedStoneTracker(
                new Point(0, 10),
                new Point(150, 200),
                0.3);
        centerStone = new FixedStoneTracker(
                new Point(180, 10),
                new Point(330, 300),
                0.3);
        rightStone = new FixedStoneTracker(
                new Point(350, 10),
                new Point(500, 200),
                0.3);

        trackerApiPipeline = new OpenCvTrackerApiPipeline();
        trackerApiPipeline.addTracker(centerStone);
        trackerApiPipeline.addTracker(leftStone);
        trackerApiPipeline.addTracker(rightStone);

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam.openCameraDevice();
        webcam.setPipeline(trackerApiPipeline);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive())
        {
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
}
