package eu.qrobotics.skystone.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

import eu.qrobotics.skystone.teamcode.subsystems.Arm;
import eu.qrobotics.skystone.teamcode.subsystems.Elevator;
import eu.qrobotics.skystone.teamcode.subsystems.FoundationGrabber.FoundationGrabberMode;
import eu.qrobotics.skystone.teamcode.subsystems.Intake;
import eu.qrobotics.skystone.teamcode.subsystems.Robot;
import eu.qrobotics.skystone.teamcode.subsystems.RuletaAlexMircea.RuletaMode;
import eu.qrobotics.skystone.teamcode.util.StickyGamepad;

import static eu.qrobotics.skystone.teamcode.subsystems.Elevator.HOLD_POWER;
import static eu.qrobotics.skystone.teamcode.subsystems.Elevator.THRESHOLD;

@TeleOp
public class TeleOP extends OpMode {
    enum DriveMode {
        NORMAL,
        SLOW,
        SUPER_SLOW
    }

    Robot robot;
    DriveMode driveMode;
    StickyGamepad stickyGamepad1 = null;
    StickyGamepad stickyGamepad2 = null;

    double elevatorDownStartTime = -100;
    double elevatorUpStartTime = -100;
    boolean elevatorToggle = true;

    @Override
    public void init() {
        robot = new Robot(this, false);
//        robot.drive.fieldCentric = true;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        driveMode = DriveMode.NORMAL;

        telemetry.log().add("Ready!");
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        switch (driveMode) {
            case NORMAL:
                robot.drive.setMotorPowersFromGamepad(gamepad1, 1);
                break;
            case SLOW:
                robot.drive.setMotorPowersFromGamepad(gamepad1, 0.7);
                break;
            case SUPER_SLOW:
                robot.drive.setMotorPowersFromGamepad(gamepad1, 0.5);
                break;
        }

        //MARK: drive speed mode
        //PRECHECK: ok
        if (stickyGamepad1.a) {
            if (driveMode != DriveMode.SLOW)
                driveMode = DriveMode.SLOW;
            else
                driveMode = DriveMode.NORMAL;
        } else if (stickyGamepad1.b) {
            if (driveMode != DriveMode.SUPER_SLOW)
                driveMode = DriveMode.SUPER_SLOW;
            else
                driveMode = DriveMode.NORMAL;
        }

        // Alex Radu
        if (stickyGamepad1.left_bumper) {
            driveMode = DriveMode.SLOW;
        }
        if (stickyGamepad1.right_bumper) {
            driveMode = DriveMode.NORMAL;
        }

        if (stickyGamepad2.x) {
            robot.arm.gripperMode = Arm.GripperMode.DROP;
        }
        if (stickyGamepad2.y) {
            robot.arm.gripperMode = Arm.GripperMode.CAPSTONE;
        }
        if (stickyGamepad2.right_bumper) {
            robot.arm.gripperMode = Arm.GripperMode.GRIP;
            elevatorUpStartTime = getRuntime();
        }
        if (stickyGamepad2.left_bumper) {
            robot.arm.gripperMode = Arm.GripperMode.DROP;
            if(robot.elevator.getTargetPosition() != Elevator.TargetHeight.STONE_15)
                robot.elevator.offsetPosition += 100;
            elevatorDownStartTime = getRuntime();
        }

        if (0.6 < getRuntime() - elevatorUpStartTime && getRuntime() - elevatorUpStartTime < 0.7) {
            robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
            elevatorToggle = false;
        }

        if (0.2 < getRuntime() - elevatorDownStartTime && getRuntime() - elevatorDownStartTime < 0.3) {
            robot.arm.armMode = Arm.ArmMode.FRONT;
        }

        if (1.0 < getRuntime() - elevatorDownStartTime && getRuntime() - elevatorDownStartTime < 1.1) {
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
            elevatorToggle = false;
        }

        if (gamepad2.left_stick_y < -0.1)
            robot.intake.intakeMode = Intake.IntakeMode.IN;
        else if (gamepad2.left_stick_y > 0.1)
            robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
        else if (robot.intake.intakeMode == Intake.IntakeMode.OUT_SLOW)
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;

        if (stickyGamepad2.dpad_up)
            robot.elevator.nextStone();
        if (stickyGamepad2.dpad_down)
            robot.elevator.previousStone();
        if (gamepad2.b)
            robot.ruleta.ruletaMode = RuletaMode.OUT;
        else if(gamepad2.a)
            robot.ruleta.ruletaMode = RuletaMode.IN;
        else
            robot.ruleta.ruletaMode = RuletaMode.IDLE;
        if(gamepad2.dpad_left)
            robot.intake.intakeMode= Intake.IntakeMode.FOLD;
        else if(robot.intake.intakeMode == Intake.IntakeMode.FOLD)
            robot.intake.intakeMode= Intake.IntakeMode.IDLE;
        if (stickyGamepad1.x || stickyGamepad1.dpad_down) {
            if (robot.foundationGrabber.foundationGrabberMode == FoundationGrabberMode.UP)
                robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.DOWN;
            else
                robot.foundationGrabber.foundationGrabberMode = FoundationGrabberMode.UP;
        }

        if (robot.elevator.elevatorMode == Elevator.ElevatorMode.UP ||
                robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
            if (gamepad2.right_trigger > 0.1) {
                robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
                robot.elevator.manualPower = gamepad2.right_trigger * 0.75;
            } else if (gamepad2.left_trigger > 0.1) {
                robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
                if(robot.elevator.getEncoder() > 1000)
                    robot.elevator.manualPower = -gamepad2.left_trigger * (-0.05);
                else
                    robot.elevator.manualPower = -gamepad2.left_trigger * 0.2;
            } else {
                if (robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
                    robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
                    robot.elevator.offsetPosition = robot.elevator.getEncoder() - robot.elevator.getTargetPosition().getEncoderPosition();
                }
            }
        }

        if (robot.elevator.elevatorMode == Elevator.ElevatorMode.UP
                && !elevatorToggle
                && Math.abs(robot.elevator.getDistanceLeft()) <= THRESHOLD + 10) {
            robot.arm.armMode = Arm.ArmMode.BACK;
            elevatorToggle = true;
        }

        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN && !elevatorToggle && Math.abs(robot.elevator.getDistanceLeft()) <= THRESHOLD) {
            robot.arm.gripperMode = Arm.GripperMode.INTAKE;
            elevatorToggle = true;
        }

        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.UP ||
                robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL ||
                Math.abs(robot.elevator.getDistanceLeft()) > THRESHOLD)
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;

        telemetry.addData("Motor Powers", Arrays.toString(robot.drive.getMotorPower()));
        telemetry.addData("Pose estimate", robot.drive.getPoseEstimate());
        telemetry.addData("Elevator Height", robot.elevator.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
