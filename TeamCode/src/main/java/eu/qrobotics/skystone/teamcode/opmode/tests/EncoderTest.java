package eu.qrobotics.skystone.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.skystone.teamcode.subsystems.Arm;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;

@TeleOp(group = "Test")
public class EncoderTest extends OpMode {
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
        telemetry.addData("Encoder Value", robot.drive.getEncoders());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}