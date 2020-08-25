package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import eu.qrobotics.skystone.teamcode.subsystems.Arm.ArmMode;

@Config
public class Intake implements Subsystem {

    public enum IntakeMode {
        IN,
        IDLE,
        OUT,
        OUT_SLOW,
        FOLD
    }

    public static double INTAKE_IN_SPEED = 1;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -0.8;
    public static double INTAKE_OUT_SLOW_SPEED = -0.4;

    public IntakeMode intakeMode;
    private DcMotorEx leftIntake, rightIntake;
    private DigitalChannel intakeButton;
    private Robot robot;
    private boolean isAutonomous;

    Intake(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        this.robot = robot;
        this.isAutonomous = isAutonomous;

        leftIntake = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        rightIntake = hardwareMap.get(DcMotorEx.class, "intakeRight");

        intakeButton = hardwareMap.get(DigitalChannel.class, "intakeButton");

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        intakeMode = IntakeMode.IDLE;
    }

    private boolean checkSwitch() {
        return intakeButton.getState();
    }

    @Override
    public void update() {
        switch (intakeMode) {
            case IN:
                if (checkSwitch()) {
                    leftIntake.setPower(INTAKE_IN_SPEED);
                    rightIntake.setPower(INTAKE_IN_SPEED);
                } else {
                    intakeMode = IntakeMode.IDLE;
                    robot.arm.armMode = ArmMode.FRONT;
                }
                break;
            case IDLE:
                leftIntake.setPower(INTAKE_IDLE_SPEED);
                rightIntake.setPower(INTAKE_IDLE_SPEED);
                break;
            case OUT:
                leftIntake.setPower(INTAKE_OUT_SPEED);
                rightIntake.setPower(INTAKE_OUT_SPEED);
                break;
            case OUT_SLOW:
                leftIntake.setPower(INTAKE_OUT_SLOW_SPEED);
                rightIntake.setPower(INTAKE_OUT_SLOW_SPEED);
                break;
            case FOLD:
                leftIntake.setPower(-0.7);
                rightIntake.setPower(0.7);
                break;
        }
    }
}
