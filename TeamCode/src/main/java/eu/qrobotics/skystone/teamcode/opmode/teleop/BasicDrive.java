package eu.qrobotics.skystone.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.skystone.teamcode.subsystems.Robot;

@TeleOp
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
}
