package eu.qrobotics.skystone.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.skystone.teamcode.subsystems.Robot;

@TeleOp(group = "Test")
public class MotorTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, false);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.drive.setMotorPowers(gamepad1.y ? 1 : 0, gamepad1.x ? 1 : 0, gamepad1.b ? 1 : 0, gamepad1.a ? 1 : 0);
    }

    @Override
    public void stop() {
        robot.stop();
    }
}