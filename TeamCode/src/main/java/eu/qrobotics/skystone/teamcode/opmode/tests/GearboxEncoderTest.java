package eu.qrobotics.skystone.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.skystone.teamcode.subsystems.Arm;
import eu.qrobotics.skystone.teamcode.subsystems.Elevator;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;

@TeleOp(group = "Test")
public class GearboxEncoderTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, false);
    }

    @Override
    public void start() {
        robot.arm.armMode = Arm.ArmMode.BACK;
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        robot.start();
    }

    @Override
    public void loop() {

        if (robot.elevator.elevatorMode == Elevator.ElevatorMode.UP ||
                robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
            if (gamepad2.right_trigger > 0.1) {
                robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
                robot.elevator.manualPower = gamepad2.right_trigger * 0.75;
            } else if (gamepad2.left_trigger > 0.1) {
                robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
                robot.elevator.manualPower = -gamepad2.left_trigger * 0.2;
            } else {
                if (robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
                    robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
                    robot.elevator.offsetPosition = robot.elevator.getEncoder() - robot.elevator.getTargetPosition().getEncoderPosition();
                }
            }
        }

        telemetry.addData("Encoder Value", robot.elevator.getEncoder());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
