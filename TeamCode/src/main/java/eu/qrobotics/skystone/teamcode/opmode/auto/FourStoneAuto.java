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
import eu.qrobotics.skystone.teamcode.subsystems.Arm;
import eu.qrobotics.skystone.teamcode.subsystems.Elevator;
import eu.qrobotics.skystone.teamcode.subsystems.FoundationGrabber.FoundationGrabberMode;
import eu.qrobotics.skystone.teamcode.subsystems.Intake;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm.ClawMode;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm.PivotMode;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.START_POSE_SIDE;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.SkystonePattern;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.getStoneIntake;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.getStoneSideArm;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.mirrorForAlliance;
import static eu.qrobotics.skystone.teamcode.subsystems.Elevator.THRESHOLD;

public abstract class FourStoneAuto extends BaseAuto {
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

    private Robot robot;

    public List<Trajectory> getTrajectories4Stones(SkystonePattern skystonePattern) {
        MultiTrajectoryBuilder multiTrajectoryBuilder = new MultiTrajectoryBuilder(getAlliance(), START_POSE_SIDE);

        int[] stonesOrder = null;
        switch (skystonePattern) {
            case LEFT:
                stonesOrder = new int[]{0, 3, 5, 4};
                break;
            case MIDDLE:
                stonesOrder = new int[]{1, 4, 5, 3};
                break;
            case RIGHT:
                stonesOrder = new int[]{3, 5, 4, 2};
                break;
        }
        Pose2d firstSkystone = getStoneSideArm(stonesOrder[0]);
        Pose2d secondSkystone = getStoneSideArm(stonesOrder[1]);
        Pose2d thirdStone = getStoneIntake(stonesOrder[2]);
        Pose2d fourthStone = getStoneIntake(stonesOrder[3]);
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(180))
                .lineToSplineHeading(firstSkystone.plus(new Pose2d(-2.5,0, 0)));
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-5, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(38, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46.5, -30.5), Math.toRadians(0));
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(5, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-5, -45), Math.toRadians(180))
                .splineToConstantHeading(secondSkystone.vec().plus(new Vector2d(1, 2.5)), Math.toRadians(180));
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-5, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(38, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46.5, -32.25), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(20, -45), () -> {
                    robot.sideArm.pivotMode = PivotMode.PLACE_UP;
                });
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(5, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-5, -45), Math.toRadians(180))
                .splineTo(thirdStone.vec(), thirdStone.getHeading())
                .addSpatialMarker(new Vector2d(-5, -45), () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                });
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -45), Math.toRadians(0))
                .splineTo(new Vector2d(57, -29), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(-5, -45), () -> {
                    Thread thread = new Thread(){
                        public void run(){
                            while(!robot.intake.checkSwitch()) {
                                robot.sleep(0.05);
                            }
                            robot.arm.gripperMode = Arm.GripperMode.GRIP;
                            robot.sleep(0.3);
                            robot.arm.armMode = Arm.ArmMode.BACK;
                        }
                    };
                    thread.start();
                });
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(-90))
                .splineTo(new Vector2d(20, -50), Math.toRadians(130))
                .splineTo(new Vector2d(-5, -45), Math.toRadians(180))
                .splineTo(fourthStone.vec().plus(new Vector2d(3, 9)), fourthStone.getHeading())
                .addSpatialMarker(new Vector2d(20, -50), () -> {
                    robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.UP;
                })
                .addSpatialMarker(new Vector2d(-5, -45), () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                });
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45, -42), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(-5, -45), () -> {
                    robot.arm.gripperMode = Arm.GripperMode.GRIP;
                })
                .addSpatialMarker(new Vector2d(10, -45), () -> {
                    robot.elevator.nextStone();
                    robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
                    robot.elevator.offsetPosition = 30;
                })
                .addDisplacementMarker(1, -4, () -> {
                    robot.arm.armMode = Arm.ArmMode.AUTONOMOUS_BACK;
                });
        multiTrajectoryBuilder.
                makeTrajectoryBuilder(MultiTrajectoryBuilder.Speed.SLOW, Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -35), Math.toRadians(180));
        return multiTrajectoryBuilder.build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.drive.setPoseEstimate(mirrorForAlliance(getAlliance(), START_POSE_SIDE));

        List<Trajectory> trajectoriesLeft = null;//getTrajectories4Stones(SkystonePattern.LEFT);
        List<Trajectory> trajectoriesMiddle = getTrajectories4Stones(SkystonePattern.MIDDLE);
        List<Trajectory> trajectoriesRight = null;//getTrajectories4Stones(SkystonePattern.RIGHT);

        /*leftStone = new FixedStoneTracker(
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
        }*/

        telemetry.addLine("READY");
        telemetry.update();
        while(!opModeIsActive() && !isStopRequested()) {
            Thread.yield();
        }

//        webcam.stopStreaming();

        if (isStopRequested())
            return;

        skystonePattern = SkystonePattern.MIDDLE;

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
        robot.sleep(0.2);
        robot.sideArm.clawMode = ClawMode.STONE;
        robot.sleep(0.2);
        robot.sideArm.pivotMode = PivotMode.MIDDLE;
        robot.sleep(0.2);

        robot.drive.followTrajectorySync(trajectories.get(1));

        //drop 1st skystone
        robot.sideArm.pivotMode = PivotMode.PLACE_DOWN;
        robot.sleep(0.15);
        robot.sideArm.clawMode = ClawMode.OPEN;
        robot.sleep(0.1);
        robot.sideArm.pivotMode = PivotMode.MIDDLE;
        robot.sleep(0.1);
        robot.sideArm.clawMode = ClawMode.STONE;

        robot.drive.followTrajectorySync(trajectories.get(2));

        //collect 2nd skystone
        robot.sideArm.clawMode = ClawMode.OPEN;
        robot.sleep(0.2);
        robot.sideArm.pivotMode = PivotMode.COLLECT;
        robot.sleep(0.2);
        robot.sideArm.clawMode = ClawMode.STONE;
        robot.sleep(0.2);
        robot.sideArm.pivotMode = PivotMode.MIDDLE;
        robot.sleep(0.2);

        robot.drive.followTrajectorySync(trajectories.get(3));

        //drop 2nd skystone
        robot.intake.intakeMode = Intake.IntakeMode.FOLD;
        robot.sideArm.clawMode = ClawMode.OPEN;
        robot.sleep(0.15);
        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        robot.sideArm.pivotMode = PivotMode.UP;
        robot.sleep(0.1);
        robot.sideArm.clawMode = ClawMode.CLOSE;

        robot.drive.followTrajectorySync(trajectories.get(4));
        robot.drive.followTrajectorySync(trajectories.get(5));

        robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.DOWN;
        robot.arm.gripperMode = Arm.GripperMode.DROP;
        robot.sleep(0.3);
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        robot.elevator.offsetPosition += 100;
        robot.sleep(0.4);
        robot.drive.followTrajectory(trajectories.get(6));
        while(robot.drive.isBusy() && !isStopRequested()) {
            if(robot.elevator.elevatorMode == Elevator.ElevatorMode.UP && Math.abs(robot.elevator.getDistanceLeft()) <= THRESHOLD) {
                robot.arm.armMode = Arm.ArmMode.FRONT;
                robot.sleep(1);
                robot.elevator.offsetPosition = 0;
                robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
                robot.sleep(1);
                robot.arm.gripperMode = Arm.GripperMode.INTAKE;
            }
            robot.sleep(0.05);
        }

        robot.drive.followTrajectorySync(trajectories.get(7));

        robot.sleep(0.2);
        robot.elevator.offsetPosition -= 20;
        robot.sleep(0.2);
        robot.arm.gripperMode = Arm.GripperMode.DROP;
        robot.sleep(0.3);
        robot.elevator.offsetPosition += 100;

        robot.drive.followTrajectory(trajectories.get(8));

        while(robot.drive.isBusy() && !isStopRequested()) {
            if(robot.elevator.elevatorMode == Elevator.ElevatorMode.UP && Math.abs(robot.elevator.getDistanceLeft()) <= THRESHOLD) {
                robot.arm.armMode = Arm.ArmMode.FRONT;
                robot.sleep(1);
                robot.elevator.offsetPosition = 0;
                robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
                robot.sleep(1);
                robot.arm.gripperMode = Arm.GripperMode.INTAKE;
            }
            robot.sleep(0.05);
        }

        robot.stop();
    }

    private double average(Scalar s) {
        if (s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}
