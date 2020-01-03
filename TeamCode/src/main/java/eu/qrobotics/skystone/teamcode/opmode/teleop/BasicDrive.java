package eu.qrobotics.skystone.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.skystone.teamcode.subsystems.Robot;

@TeleOp
@Disabled
public class BasicDrive extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, false);

        telemetry.log().add("Initialized!");
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.drive.setMotorPowersFromGamepad(gamepad1, 1);
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }

    private void addStatistics() {
        telemetry.addData("Top 250", formatResults(robot.top250));
        telemetry.addData("Top 100", formatResults(robot.top100));
        telemetry.addData("Top 10", formatResults(robot.top10));
    }
}
