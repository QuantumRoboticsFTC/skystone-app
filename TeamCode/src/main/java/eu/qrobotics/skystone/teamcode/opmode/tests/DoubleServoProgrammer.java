package eu.qrobotics.skystone.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.skystone.teamcode.util.StickyGamepad;

@TeleOp(name = "Double Servo Programmer", group = "Test")
public class DoubleServoProgrammer extends OpMode {
    enum ProgrammerMode {
        Low, Medium, High;

        public double getRawValue() {
            switch (this) {
                case Low:
                    return 0.001;
                case Medium:
                    return 0.025;
                case High:
                    return 0.05;
                default:
                    return 0.05;
            }
        }

        public ProgrammerMode nextMode() {
            switch (this) {
                case Low:
                    return ProgrammerMode.Medium;
                case Medium:
                    return ProgrammerMode.High;
                case High:
                    return ProgrammerMode.Low;
                default:
                    return ProgrammerMode.High;

            }
        }

        public String stringValue() {
            switch (this) {
                case Low:
                    return "Low";
                case Medium:
                    return "Medium";
                case High:
                    return "High";
                default:
                    return "Unknown";
            }
        }
    }

    private Servo leftServo = null;
    private Servo rightServo = null;

    private StickyGamepad stickyGamepad = null;

    private boolean isLeftServoDisabled = true;
    private boolean isRightServoDisabled = true;

    private ProgrammerMode programmerMode = ProgrammerMode.High;

    // initial servo positions
    private double currentPositionLeft = 0.862;
    private double currentPositionRight = 0.138;

    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightServo = hardwareMap.get(Servo.class, "rightArmServo");

        stickyGamepad = new StickyGamepad(gamepad1);

        telemetry.addData("Initial left Position", currentPositionLeft);
        telemetry.addData("Initial right Position", currentPositionRight);
    }

    @Override
    public void loop() {
        stickyGamepad.update();

        // set precision
        if (stickyGamepad.x) {
            programmerMode = programmerMode.nextMode();
        }

        // set position to both servos
        if (stickyGamepad.right_bumper) {
            currentPositionLeft -= programmerMode.getRawValue();
            currentPositionRight += programmerMode.getRawValue();
        } else if (stickyGamepad.left_bumper) {
            currentPositionLeft += programmerMode.getRawValue();
            currentPositionRight -= programmerMode.getRawValue();
        }

        updateLeftServo();
        updateRightServo();

        updateTelemetry();
    }

    private void updateLeftServo() {
        // set servo position
        if (stickyGamepad.dpad_down && currentPositionLeft >= programmerMode.getRawValue()) {
            currentPositionLeft -= programmerMode.getRawValue();
        } else if (stickyGamepad.dpad_up && currentPositionLeft + programmerMode.getRawValue() <= 1) {
            currentPositionLeft += programmerMode.getRawValue();
        }

        // toggle pwm
        if (stickyGamepad.dpad_right) {
            isLeftServoDisabled = !isLeftServoDisabled;

            // update pwm and servo position
            if (isLeftServoDisabled) {
                leftServo.getController().pwmDisable();
            } else {
                leftServo.getController().pwmEnable();
            }
        }

        leftServo.setPosition(currentPositionLeft);
    }

    private void updateRightServo() {
        // set servo position
        if (stickyGamepad.a && currentPositionRight >= programmerMode.getRawValue()) {
            currentPositionRight -= programmerMode.getRawValue();
        } else if (stickyGamepad.y && currentPositionRight + programmerMode.getRawValue() <= 1) {
            currentPositionRight += programmerMode.getRawValue();
        }

        // toggle pwm
        if (stickyGamepad.b) {
            isRightServoDisabled = !isRightServoDisabled;

            // update pwm and servo position
            if (isRightServoDisabled) {
                rightServo.getController().pwmDisable();
            } else {
                rightServo.getController().pwmEnable();

            }
        }

        rightServo.setPosition(currentPositionRight);
    }

    private void updateTelemetry() {
        telemetry.addData("Programmer Mode", programmerMode.stringValue());
        telemetry.addData("Precision", programmerMode.getRawValue());
        telemetry.addData("Left Servo Position", leftServo.getPosition());
        telemetry.addData("Right Servo Position", rightServo.getPosition());
        telemetry.addData("Left Servo Running", !isLeftServoDisabled);
        telemetry.addData("Right Servo Running", !isRightServoDisabled);
        telemetry.update();
    }

}

//first stone position

//left: 862
//right: 138


//brat

//0.385 horizontal
//00 up


//claw
//00 fully closed
//1 fully open
//0.25 stonk

