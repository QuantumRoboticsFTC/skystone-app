package eu.qrobotics.skystone.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.skystone.teamcode.subsystems.Elevator.ElevatorMode;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;

@TeleOp(group = "Test")
public class ElevatorManualTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, false);
        robot.elevator.elevatorMode = ElevatorMode.MANUAL;
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.elevator.manualPower = gamepad1.right_trigger - gamepad1.left_trigger;
        telemetry.addData("Encoder Value", robot.elevator.getEncoder());
        telemetry.addData("Raw Encoder Value", robot.elevator.getRawEncoder());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
