package eu.qrobotics.skystone.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.skystone.teamcode.subsystems.Intake;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;

@TeleOp(group = "Test")
public class IntakeTest extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, false);
        telemetry.log().add("Ready!");
    }

    @Override
    public void start() {
        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        robot.start();
    }

    @Override
    public void loop() {
        if (gamepad1.a)
            robot.intake.intakeMode = Intake.IntakeMode.IN;
        if(gamepad1.b) {
            robot.intake.intakeMode= Intake.IntakeMode.IDLE;
        }
        if(gamepad1.y) {
            robot.intake.intakeMode= Intake.IntakeMode.OUT;
        }
        telemetry.addData("INTAKE MODE", robot.intake.intakeMode);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
