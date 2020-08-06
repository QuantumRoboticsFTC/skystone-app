package eu.qrobotics.skystone.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.skystone.teamcode.subsystems.Arm;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;
import eu.qrobotics.skystone.teamcode.util.StickyGamepad;

@TeleOp(group = "Test")
public class ArmTest extends OpMode {
    Robot robot;
    StickyGamepad gamepad;


    @Override
    public void init() {
        robot = new Robot(this, false);
        gamepad = new StickyGamepad(gamepad1);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.b) {
            if (robot.arm.armMode == Arm.ArmMode.FRONT)
                robot.arm.armMode = Arm.ArmMode.BACK;
            else
                robot.arm.armMode = Arm.ArmMode.FRONT;
        }

        if (gamepad.x) {
            if (robot.arm.gripperMode == Arm.GripperMode.DROP)
                robot.arm.gripperMode = Arm.GripperMode.GRIP;
            else
                robot.arm.gripperMode = Arm.GripperMode.DROP;
        }
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
