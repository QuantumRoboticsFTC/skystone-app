package eu.qrobotics.skystone.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.skystone.teamcode.subsystems.Arm;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm;
import eu.qrobotics.skystone.teamcode.subsystems.SideArm.PivotMode;
import eu.qrobotics.skystone.teamcode.util.StickyGamepad;

import static eu.qrobotics.skystone.teamcode.subsystems.SideArm.*;

@TeleOp(group = "Test")
@Config
public class SideArmTest extends LinearOpMode {
    Robot robot;
    StickyGamepad gamepad;

    public static double CLAW_SLEEP = 0.5;
    public static double UP_SLEEP = 0.4;
    public static double DOWN_SLEEP = 0.3;
    public static double DROP_SLEEP = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, false);
        gamepad = new StickyGamepad(gamepad1);

        waitForStart();
        robot.start();
        robot.sleep(1);

        while(!isStopRequested() && opModeIsActive()) {
            gamepad.update();

            robot.sideArm.pivotMode = PivotMode.COLLECT;
            robot.sideArm.clawMode = ClawMode.STONE;
            robot.sleep(CLAW_SLEEP);
            robot.sideArm.pivotMode = PivotMode.MIDDLE;
            robot.sleep(UP_SLEEP);

            robot.sideArm.pivotMode = PivotMode.PLACE_DOWN;
            robot.sleep(DOWN_SLEEP);
            robot.sideArm.clawMode = ClawMode.OPEN;
            robot.sleep(DROP_SLEEP);
            robot.sideArm.pivotMode = PivotMode.MIDDLE;
            robot.sideArm.clawMode = ClawMode.OPEN;

            robot.sleep(2);
        }

        robot.stop();
    }
}
