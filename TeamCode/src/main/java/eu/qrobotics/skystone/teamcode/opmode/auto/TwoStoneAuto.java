package eu.qrobotics.skystone.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.visualizer.MultiTrajectoryBuilder;

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
import eu.qrobotics.skystone.teamcode.subsystems.FoundationGrabber.FoundationGrabberMode;
import eu.qrobotics.skystone.teamcode.subsystems.Intake;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm.ClawMode;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm.PivotMode;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.*;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.START_POSE_SIDE;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.mirrorForAlliance;

public abstract class TwoStoneAuto extends BaseAuto {
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

    private List<Trajectory> getTrajectoriesSideArm2Stones(SkystonePattern skystonePattern) {
        MultiTrajectoryBuilder multiTrajectoryBuilder = new MultiTrajectoryBuilder(getAlliance(), START_POSE_SIDE);

        int[] stonesOrder = null;
        switch (skystonePattern) {
            case LEFT:
                stonesOrder = new int[]{0, 3};
                break;
            case MIDDLE:
                stonesOrder = new int[]{1, 4};
                break;
            case RIGHT:
                stonesOrder = new int[]{3, 5};
                break;
        }
        Pose2d firstSkystone = getStoneSideArm(stonesOrder[0]);
        Pose2d secondSkystone = getStoneSideArm(stonesOrder[1]);
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(180))
                .lineToSplineHeading(firstSkystone.plus(new Pose2d(-3.5, 0, 0)));
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-5, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(51.5, -33), Math.toRadians(0));
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(5, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-5, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(secondSkystone.getX(), secondSkystone.getY()), Math.toRadians(180));
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-5, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(51.5, -35.5), Math.toRadians(0));
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(51.5, -43, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(60, -28), Math.toRadians(90));
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(20, -45, Math.toRadians(110)), Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(43, -60, Math.toRadians(180)), Math.toRadians(0));
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, -35, Math.toRadians(180)), Math.toRadians(180))
                .splineTo(new Vector2d(5, -35), Math.toRadians(180));
        return multiTrajectoryBuilder.build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true);
        robot.drive.setPoseEstimate(mirrorForAlliance(getAlliance(), START_POSE_SIDE));

        List<Trajectory> trajectoriesLeft = getTrajectoriesSideArm2Stones(SkystonePattern.LEFT);
        List<Trajectory> trajectoriesMiddle = getTrajectoriesSideArm2Stones(SkystonePattern.MIDDLE);
        List<Trajectory> trajectoriesRight = getTrajectoriesSideArm2Stones(SkystonePattern.RIGHT);

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

        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        while (!opModeIsActive() && !isStopRequested()) {
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

            telemetry.addData("Skystone Pattern", skystonePattern);
            telemetry.update();
        }

//        webcam.stopStreaming();

        if (isStopRequested())
            return;

        if (getAlliance() == Alliance.BLUE) {
            if (skystonePattern == SkystonePattern.LEFT)
                skystonePattern = SkystonePattern.RIGHT;
            else if (skystonePattern == SkystonePattern.RIGHT)
                skystonePattern = SkystonePattern.LEFT;
        }

        List<Trajectory> trajectories = null;
        switch(skystonePattern) {
            case MIDDLE:
                trajectories = trajectoriesMiddle;
                break;
            case LEFT:
                if(getAlliance() == Alliance.BLUE)
                    trajectories = trajectoriesRight;
                else
                    trajectories = trajectoriesLeft;
                break;
            case RIGHT:
                if(getAlliance() == Alliance.BLUE)
                    trajectories = trajectoriesLeft;
                else
                    trajectories = trajectoriesRight;
                break;
        }

        robot.start();

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
        if (s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}
