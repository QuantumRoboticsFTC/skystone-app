package eu.qrobotics.skystone.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.AutoTrajectoryGenerator;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.AutoTrajectoryGenerator.SkystonePattern;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils;
import eu.qrobotics.skystone.teamcode.subsystems.FoundationGrabber;
import eu.qrobotics.skystone.teamcode.subsystems.FoundationGrabber.FoundationGrabberMode;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm.ClawMode;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm.PivotMode;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.START_POSE_SIDE;

public abstract class TwoStoneAuto extends LinearOpMode {
    abstract TrajectoryUtils.Alliance getAlliance();

    SkystonePattern skystonePattern = SkystonePattern.MIDDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true);
        AutoTrajectoryGenerator trajectoryGenerator = new AutoTrajectoryGenerator(getAlliance(), START_POSE_SIDE);
        robot.drive.setPoseEstimate(START_POSE_SIDE);

        waitForStart();

        robot.start();
        List<Trajectory> trajectories = trajectoryGenerator.getTrajectoriesSideArm6Stones(skystonePattern);

        robot.drive.followTrajectorySync(trajectories.get(0));

        robot.sideArm.pivotMode = PivotMode.DOWN;
        robot.sleep(0.6);
        robot.sideArm.clawMode = ClawMode.STONE;
        robot.sleep(0.8);
        robot.sideArm.pivotMode = PivotMode.MIDDLE;
        robot.sleep(0.6);

        robot.drive.followTrajectorySync(trajectories.get(1));

        robot.sideArm.pivotMode = PivotMode.DOWN;
        robot.sleep(0.3);
        robot.sideArm.clawMode = ClawMode.OPEN;
        robot.sleep(0.3);
        robot.sideArm.pivotMode = PivotMode.UP;
        robot.sideArm.clawMode = ClawMode.STONE;

        robot.drive.followTrajectorySync(trajectories.get(2));

        robot.sideArm.clawMode = ClawMode.OPEN;
        robot.sideArm.pivotMode = PivotMode.DOWN;
        robot.sleep(0.6);
        robot.sideArm.clawMode = ClawMode.STONE;
        robot.sleep(0.8);
        robot.sideArm.pivotMode = PivotMode.MIDDLE;
        robot.sleep(0.6);

        robot.drive.followTrajectorySync(trajectories.get(3));

        //robot.sideArm.pivotMode = PivotMode.DOWN;
        //robot.sleep(0.3);
        robot.sideArm.clawMode = ClawMode.OPEN;
        robot.sleep(0.3);
        robot.sideArm.pivotMode = PivotMode.UP;

        robot.drive.followTrajectorySync(trajectories.get(4));

        robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.DOWN;
        robot.sleep(1);

        robot.drive.followTrajectorySync(trajectories.get(5));

        robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.UP;
        robot.sleep(1);

        robot.drive.followTrajectorySync(trajectories.get(6));

        robot.stop();
    }
}
