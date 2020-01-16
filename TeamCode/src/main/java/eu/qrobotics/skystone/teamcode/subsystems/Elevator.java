package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class Elevator implements Subsystem {

    public static int THRESHOLD_DOWN = 20;
    public static int THRESHOLD = 10;
    public static int THRESHOLD_LEVEL_1 = 40;
    public static int THRESHOLD_LEVEL_2 = 100;
    public static int THRESHOLD_LEVEL_3 = 250;
    public static double DOWN_POWER = -0.85;
    public static double HOLD_POWER = 0.1;
    public static double LEVEL_1_POWER = 0.2;
    public static double LEVEL_2_POWER = 0.5;
    public static double LEVEL_3_POWER = 0.85;
    public static double LEVEL_4_POWER = 1;

    public enum ElevatorMode {
        DISABLED,
        DOWN,
        UP,
        MANUAL
    }

    public enum TargetHeight {
        STONE_1(110),
        STONE_2(310),
        STONE_3(510),
        STONE_4(710),
        STONE_5(910),
        STONE_6(1110),
        STONE_7(1310),
        STONE_8(1510),
        STONE_9(1710),
        STONE_10(1910),
        STONE_11(2110),
        STONE_12(2310),
        STONE_13(2510),
        STONE_14(2710);

        private final int encoderPosition;

        TargetHeight(int encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public int getEncoderPosition() {
            return encoderPosition;
        }
    }

    private int downPosisition;
    private int lastEncoder;
    public double offsetPosition;
    private TargetHeight targetPosition;
    public double manualPower;

    public ElevatorMode elevatorMode;
    private ExpansionHubMotor motorLeft, motorRight;
    private Robot robot;

    Elevator(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        this.robot = robot;

        motorLeft = hardwareMap.get(ExpansionHubMotor.class, "elevatorLeft");
        motorRight = hardwareMap.get(ExpansionHubMotor.class, "elevatorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        //motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        downPosisition = getRawEncoder();

        elevatorMode = ElevatorMode.DISABLED;
        targetPosition = TargetHeight.STONE_1;
    }

    public int getRawEncoder() {
        return -motorRight.getCurrentPosition(); // left
        //return motorRight.getCurrentPosition(); // right
    }

    public int getEncoder() {
        lastEncoder = getRawEncoder() - downPosisition;
        return lastEncoder;
    }

    public int getLastEncoder() {
        return lastEncoder;
    }

    public int getDistanceLeft() {
        return targetPosition.getEncoderPosition() - lastEncoder + (int)offsetPosition;
    }

    public TargetHeight getTargetPosition() {
        return targetPosition;
    }

    private void setPower(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    @Override
    public void update() {
        if (elevatorMode == ElevatorMode.DISABLED)
            return;

        if (elevatorMode == ElevatorMode.DOWN) {
            offsetPosition = 0;
            if (getEncoder() <= THRESHOLD_DOWN)
                setPower(0);
            else
                setPower(DOWN_POWER);
        } else if (elevatorMode == ElevatorMode.UP) {
            int distanceLeft = targetPosition.getEncoderPosition() - getEncoder() + (int)offsetPosition;
            if (Math.abs(distanceLeft) <= THRESHOLD)
                setPower(HOLD_POWER);
            else if (Math.abs(distanceLeft) <= THRESHOLD_LEVEL_1)
                setPower(LEVEL_1_POWER * Math.signum(distanceLeft));
            else if (Math.abs(distanceLeft) <= THRESHOLD_LEVEL_2)
                setPower(LEVEL_2_POWER * Math.signum(distanceLeft));
            else if (Math.abs(distanceLeft) <= THRESHOLD_LEVEL_3)
                setPower(LEVEL_3_POWER * Math.signum(distanceLeft));
            else
                setPower(LEVEL_4_POWER * Math.signum(distanceLeft));
        } else {
            setPower(manualPower);
        }
    }

    public void nextStone() {
        switch (targetPosition) {
            case STONE_1:
                targetPosition = TargetHeight.STONE_2;
                break;
            case STONE_2:
                targetPosition = TargetHeight.STONE_3;
                break;
            case STONE_3:
                targetPosition = TargetHeight.STONE_4;
                break;
            case STONE_4:
                targetPosition = TargetHeight.STONE_5;
                break;
            case STONE_5:
                targetPosition = TargetHeight.STONE_6;
                break;
            case STONE_6:
                targetPosition = TargetHeight.STONE_7;
                break;
            case STONE_7:
                targetPosition = TargetHeight.STONE_8;
                break;
            case STONE_8:
                targetPosition = TargetHeight.STONE_9;
                break;
            case STONE_9:
                targetPosition = TargetHeight.STONE_10;
                break;
            case STONE_10:
                targetPosition = TargetHeight.STONE_11;
                break;
            case STONE_11:
                targetPosition = TargetHeight.STONE_12;
                break;
            case STONE_12:
                targetPosition = TargetHeight.STONE_13;
                break;
            case STONE_13:
                targetPosition = TargetHeight.STONE_14;
                break;
            case STONE_14:
                break;
        }
    }

    public void previousStone() {
        switch (targetPosition) {
            case STONE_1:
                break;
            case STONE_2:
                targetPosition = TargetHeight.STONE_1;
                break;
            case STONE_3:
                targetPosition = TargetHeight.STONE_2;
                break;
            case STONE_4:
                targetPosition = TargetHeight.STONE_3;
                break;
            case STONE_5:
                targetPosition = TargetHeight.STONE_4;
                break;
            case STONE_6:
                targetPosition = TargetHeight.STONE_5;
                break;
            case STONE_7:
                targetPosition = TargetHeight.STONE_6;
                break;
            case STONE_8:
                targetPosition = TargetHeight.STONE_7;
                break;
            case STONE_9:
                targetPosition = TargetHeight.STONE_8;
                break;
            case STONE_10:
                targetPosition = TargetHeight.STONE_9;
                break;
            case STONE_11:
                targetPosition = TargetHeight.STONE_10;
                break;
            case STONE_12:
                targetPosition = TargetHeight.STONE_11;
                break;
            case STONE_13:
                targetPosition = TargetHeight.STONE_12;
                break;
            case STONE_14:
                targetPosition = TargetHeight.STONE_13;
                break;
        }
    }
}
