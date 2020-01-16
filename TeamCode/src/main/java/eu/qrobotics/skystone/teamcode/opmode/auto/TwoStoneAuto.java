package eu.qrobotics.skystone.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;

import java.util.Arrays;
import java.util.List;

import eu.qrobotics.skystone.teamcode.cv.trackers.FixedStoneTracker;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.AutoTrajectoryGenerator;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.AutoTrajectoryGenerator.SkystonePattern;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils;
import eu.qrobotics.skystone.teamcode.subsystems.FoundationGrabber.FoundationGrabberMode;
import eu.qrobotics.skystone.teamcode.subsystems.Intake;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm.ClawMode;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm.PivotMode;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.START_POSE_SIDE;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.flipIfBlue;

public abstract class TwoStoneAuto extends LinearOpMode {
    abstract TrajectoryUtils.Alliance getAlliance();

    SkystonePattern skystonePattern = SkystonePattern.MIDDLE;

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

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true);
        AutoTrajectoryGenerator trajectoryGenerator = new AutoTrajectoryGenerator(getAlliance(), START_POSE_SIDE);
        robot.drive.setPoseEstimate(flipIfBlue(getAlliance(), START_POSE_SIDE));

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
            double[] counts = {average(leftStone.getCount()),
                    average(centerStone.getCount()),
                    average(rightStone.getCount())};

            int maxIdx = 0;
            double max = 0;
            for (int i = 0; i < counts.length; i++) {
                if(counts[i] > max) {
                    max = counts[i];
                    maxIdx = i;
                }
            }

            if(maxIdx == 0) {
                skystonePattern = SkystonePattern.LEFT;
            } else if(maxIdx == 1) {
                skystonePattern = SkystonePattern.MIDDLE;
            } else {
                skystonePattern = SkystonePattern.RIGHT;
            }

            telemetry.addData("Skystone Pattern", skystonePattern);
            telemetry.update();
        }

        webcam.stopStreaming();

        if(isStopRequested())
            return;

        if(getAlliance() == TrajectoryUtils.Alliance.BLUE) {
            if(skystonePattern == SkystonePattern.LEFT)
                skystonePattern = SkystonePattern.RIGHT;
            else if(skystonePattern == SkystonePattern.RIGHT)
                skystonePattern = SkystonePattern.LEFT;
        }

        robot.start();
        List<Trajectory> trajectories = trajectoryGenerator.getTrajectoriesSideArm2Stones(skystonePattern);

        robot.drive.followTrajectorySync(trajectories.get(0));

        //collect 1st skystone
        robot.sideArm.pivotMode = PivotMode.COLLECT;
        robot.sleep(0.35);
        robot.sideArm.clawMode = ClawMode.STONE;
        robot.sleep(0.4);
        robot.sideArm.pivotMode = PivotMode.MIDDLE;
        robot.sleep(0.5);

        robot.drive.followTrajectorySync(trajectories.get(1));

        //drop 1st skystone
        robot.sideArm.pivotMode = PivotMode.PLACE_DOWN;
        robot.sleep(0.2);
        robot.sideArm.clawMode = ClawMode.OPEN;
        robot.sleep(0.2);
        robot.sideArm.pivotMode = PivotMode.MIDDLE;
        robot.sleep(0.2);
        robot.sideArm.clawMode = ClawMode.STONE;

        robot.drive.followTrajectorySync(trajectories.get(2));

        //collect 2nd skystone
        robot.sideArm.clawMode = ClawMode.OPEN;
        robot.sleep(0.6);
        robot.sideArm.pivotMode = PivotMode.COLLECT;
        robot.sleep(0.35);
        robot.sideArm.clawMode = ClawMode.STONE;
        robot.sleep(0.4);
        robot.sideArm.pivotMode = PivotMode.MIDDLE;
        robot.sleep(0.5);

        robot.drive.followTrajectorySync(trajectories.get(3));

        //drop 2nd skystone
        robot.sideArm.pivotMode = PivotMode.PLACE_UP;
        robot.sleep(0.2);
        robot.sideArm.clawMode = ClawMode.OPEN;
        robot.sleep(0.2);
        robot.sideArm.pivotMode = PivotMode.UP;
        robot.sleep(0.2);
        robot.sideArm.clawMode = ClawMode.CLOSE;

        robot.drive.followTrajectorySync(trajectories.get(4));

        //grab foundation
        robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.DOWN;
        robot.sleep(1);

        robot.drive.followTrajectorySync(trajectories.get(5));

        //release foundation
        robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.UP;
        robot.sleep(1);

        robot.intake.intakeMode = Intake.IntakeMode.FOLD;

        robot.drive.followTrajectorySync(trajectories.get(6));


        robot.stop();
    }

    private double average(Scalar s) {
        if(s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}
