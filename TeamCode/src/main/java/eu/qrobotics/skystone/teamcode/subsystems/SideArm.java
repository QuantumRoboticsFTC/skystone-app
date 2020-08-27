package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SideArm implements Subsystem {

    public enum PivotMode {
        START,
        UP,
        MIDDLE,
        COLLECT,
        PLACE_UP,
        PLACE_DOWN
    }

    public enum ClawMode {
        CLOSE,
        STONE,
        OPEN
    }

    public static double PIVOT_START = 0.56;
    public static double PIVOT_UP = 0.56;
    public static double PIVOT_MIDDLE = 0.45;
    public static double PIVOT_COLLECT = 0.10;
    public static double PIVOT_PLACE_UP = 0.22;
    public static double PIVOT_PLACE_DOWN = 0.15;

    public static double CLAW_CLOSE = 0;
    public static double CLAW_STONE = 0.6;
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

        pivotMode = PivotMode.START;
        if (isAutonomous)
            clawMode = ClawMode.OPEN;
        else
            clawMode = ClawMode.CLOSE;
    }

    @Override
    public void update() {
        switch (pivotMode) {
            case START:
                pivotServo.setPosition(PIVOT_START);
                break;
            case UP:
                pivotServo.setPosition(PIVOT_UP);
                break;
            case MIDDLE:
                pivotServo.setPosition(PIVOT_MIDDLE);
                break;
            case COLLECT:
                pivotServo.setPosition(PIVOT_COLLECT);
                break;
            case PLACE_UP:
                pivotServo.setPosition(PIVOT_PLACE_UP);
                break;
            case PLACE_DOWN:
                pivotServo.setPosition(PIVOT_PLACE_DOWN);
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
