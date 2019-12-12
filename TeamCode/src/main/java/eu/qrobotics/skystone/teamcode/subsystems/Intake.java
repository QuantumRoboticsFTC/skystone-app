package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class Intake implements Subsystem {

    public enum IntakeMode {
        IN,
        IDLE,
        OUT,
        OUT_SLOW
    }

    public static double INTAKE_IN_SPEED = 0.9;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -0.8;
    public static double INTAKE_OUT_SLOW_SPEED = -0.4;

    public IntakeMode intakeMode;
    private ExpansionHubMotor leftIntake, rightIntake;
    private Robot robot;

    Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntakeMotor");

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void update() {
        switch (intakeMode) {
            case IN:
                leftIntake.setPower(INTAKE_IN_SPEED);
                rightIntake.setPower(INTAKE_IN_SPEED);
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
        }
    }
}
