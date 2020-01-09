package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SideArm implements Subsystem {

    public enum PivotMode {
        UP,
        MIDDLE,
        DOWN
    }

    public enum ClawMode {
        CLOSE,
        STONE,
        OPEN
    }

    public static double PIVOT_UP = 0.015;
    public static double PIVOT_MIDDLE = 0.115;
    public static double PIVOT_DOWN = 0.4;

    public static double CLAW_CLOSE = 0;
    public static double CLAW_STONE = 0.25;
    public static double CLAW_OPEN = 1;

    public PivotMode pivotMode;
    public ClawMode clawMode;

    private Servo pivotServo, clawServo;
    private Robot robot;
    private boolean isAutonomous;

    public SideArm(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        this.isAutonomous = isAutonomous;
        this.robot = robot;

        pivotServo = hardwareMap.get(Servo.class, "sideArmPivot");
        clawServo = hardwareMap.get(Servo.class, "sideArmClaw");

        pivotMode = PivotMode.UP;
        if (isAutonomous)
            clawMode = ClawMode.OPEN;
        else
            clawMode = ClawMode.CLOSE;
    }

    @Override
    public void update() {
        switch (pivotMode) {
            case UP:
                pivotServo.setPosition(PIVOT_UP);
                break;
            case MIDDLE:
                pivotServo.setPosition(PIVOT_MIDDLE);
                break;
            case DOWN:
                pivotServo.setPosition(PIVOT_DOWN);
                break;
        }

        switch (clawMode) {
            case CLOSE:
                clawServo.setPosition(CLAW_CLOSE);
                break;
            case STONE:
                clawServo.setPosition(CLAW_STONE);
                break;
            case OPEN:
                clawServo.setPosition(CLAW_OPEN);
                break;
        }
    }
}
