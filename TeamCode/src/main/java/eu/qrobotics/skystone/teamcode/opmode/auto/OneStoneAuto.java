package eu.qrobotics.skystone.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;

import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.AutoTrajectoryGenerator;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.AutoTrajectoryGenerator.SkystonePattern;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;
import eu.qrobotics.skystone.teamcode.subsystems.Arm.ArmMode;
import eu.qrobotics.skystone.teamcode.subsystems.Arm.GripperMode;
import eu.qrobotics.skystone.teamcode.subsystems.DriveConstants;
import eu.qrobotics.skystone.teamcode.subsystems.Elevator.ElevatorMode;
import eu.qrobotics.skystone.teamcode.subsystems.FoundationGrabber.FoundationGrabberMode;
import eu.qrobotics.skystone.teamcode.subsystems.Intake.IntakeMode;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.flipIfBlue;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public abstract class OneStoneAuto extends LinearOpMode {
    public static Pose2d STARTING_POSE = new Pose2d(-33, -63, Math.toRadians(90));

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AeS98kj/////AAABmXdQauRe0EckiIHBShZZ7Zwq2addwm/e1hNj2f5q5ApkyqGI7LhjE/c37puxUeyyCluEik9kbr+hEVPPL3iGlKbOaAy+6UmjaM7zXtsXEyoPjT9KdbAPlVbSbZ/FRUuQHa+fUCiNLOmTyD5EURAtR+hGV2q+IUvTt9jAeITP9NbhX0HWBa5l6DEp1noOvvug8VElcVjriGr6eiSdprFh7tPsPki+MO4oz8uveRCq5coebi3ET4BfRjYnmjx4xyaRlkh+P0xwpvNRkXuwH+lXjT1eyat+MywP2DEQBzjT+DZ7EncIT1qhQVKHfydX0ECTpR6jJsT70x6z8/Wg3V13UTug/bqVfloGMUEsILRbGJd5";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final float stoneZ = 2.00f * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;

    abstract Alliance getAlliance();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true);
        AutoTrajectoryGenerator trajectoryGenerator = new AutoTrajectoryGenerator(getAlliance(), STARTING_POSE);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. For OpenRC, these are loaded from
        // the internal storage to reduce APK size
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromFile("/sdcard/FIRST/Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        targetsSkyStone.activate();

        SkystonePattern skystonePattern = getAlliance() == Alliance.RED ? SkystonePattern.RIGHT : SkystonePattern.LEFT;

        while (!isStarted() && !isStopRequested()) {
            targetVisible = false;
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                double position = translation.get(0) / mmPerInch;
                if (0 < position)
                    skystonePattern = SkystonePattern.MIDDLE;
                else
                    skystonePattern = getAlliance() == Alliance.RED ? SkystonePattern.LEFT : SkystonePattern.RIGHT;
            } else
                skystonePattern = getAlliance() == Alliance.RED ? SkystonePattern.RIGHT : SkystonePattern.LEFT;

            telemetry.addData("Skystone", skystonePattern);
            telemetry.update();
        }

        robot.intake.intakeMode = IntakeMode.FOLD;
        robot.start();
        robot.drive.setPoseEstimate(flipIfBlue(getAlliance(), STARTING_POSE));

        robot.drive.followTrajectorySync(
                new TrajectoryBuilder(flipIfBlue(getAlliance(), STARTING_POSE), DriveConstants.SLOW_CONSTRAINTS)
                        .forward(17)
                        .build());

        double time = getRuntime();
        skystonePattern = getAlliance() == Alliance.RED ? SkystonePattern.RIGHT : SkystonePattern.LEFT;

        while (getRuntime() - time < 0.5) {
            targetVisible = false;
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getFtcCameraFromTarget();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                double position = translation.get(0) / mmPerInch;
                if (0 < position)
                    skystonePattern = getAlliance() == TrajectoryUtils.Alliance.RED ? AutoTrajectoryGenerator.SkystonePattern.LEFT : AutoTrajectoryGenerator.SkystonePattern.RIGHT;
                else
                    skystonePattern = AutoTrajectoryGenerator.SkystonePattern.MIDDLE;
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        robot.drive.followTrajectorySync(new TrajectoryBuilder(robot.drive.getPoseEstimate(), DriveConstants.SLOW_CONSTRAINTS).back(17).build());

        List<Trajectory> trajectories = trajectoryGenerator.getTrajectories1Stone(skystonePattern);
        targetsSkyStone.deactivate();

        robot.elevator.elevatorMode = ElevatorMode.DISABLED;
        robot.arm.armMode = ArmMode.FRONT;
        robot.intake.intakeMode = IntakeMode.IN;

        robot.drive.followTrajectorySync(trajectories.get(0));

        robot.sleep(0.5);

        robot.drive.followTrajectorySync(trajectories.get(1));

        robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.DOWN;
        robot.sleep(1);

        robot.drive.followTrajectorySync(trajectories.get(2));

        robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.UP;
        robot.sleep(0.5);
        robot.arm.armMode = ArmMode.BACK;
        robot.sleep(1);

        robot.arm.gripperMode = GripperMode.DROP;
        robot.sleep(0.5);
        robot.arm.armMode = ArmMode.FRONT;
        robot.sleep(1);

        robot.drive.followTrajectorySync(trajectories.get(3));
        robot.sleep(2);

        robot.stop();
    }
}
