package eu.qrobotics.skystone.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import eu.qrobotics.skystone.teamcode.subsystems.DriveConstants;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;

@Autonomous
public class JustForward extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, true);
    }

    @Override
    public void start() {
        robot.start();
        robot.drive.followTrajectorySync(new TrajectoryBuilder(robot.drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS).forward(8).build());
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        robot.stop();
    }
}
