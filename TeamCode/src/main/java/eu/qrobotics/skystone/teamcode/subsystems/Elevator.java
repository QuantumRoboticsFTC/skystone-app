package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Elevator implements Subsystem {

    public static int THRESHOLD_DOWN = 20;
    public static int THRESHOLD = 10;
    public static int THRESHOLD_LEVEL_1 = 10;
    public static int THRESHOLD_LEVEL_2 = 50;
    public static int THRESHOLD_LEVEL_3 = 250;
    public static double DOWN_POWER = -0.35;
    public static double HOLD_POWER = 0.21;
    public static double LEVEL_1_POWER = 0.4;
    public static double LEVEL_2_POWER = 0.6;
    public static double LEVEL_3_POWER = 0.85;
    public static double LEVEL_4_POWER = 1;

    public enum ElevatorMode {
        DISABLED,
        DOWN,
        UP,
        MANUAL
    }

    public enum TargetHeight {
        STONE_1(0) {
            @Override
            public TargetHeight previous() {
                return this;
            }
        },
        STONE_2(120),
        STONE_3(250),
        STONE_4(380),
        STONE_5(510),
        STONE_6(640),
        STONE_7(770),
        STONE_8(900),
        STONE_9(1030),
        STONE_10(1160),
        STONE_11(1290),
        STONE_12(1420),
        STONE_13(1540),
        STONE_14(1650),
        STONE_15(1740) {
            @Override
            public TargetHeight next() {
                return this;
            }
        };

        private final int encoderPosition;

        TargetHeight(int encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public int getEncoderPosition() {
            return encoderPosition;
        }

        public TargetHeight previous() {
            return values()[ordinal() - 1];
        }

        public TargetHeight next() {
            return values()[ordinal() + 1];
        }
    }

    private int downPosition;
    private int lastEncoder;
    public double offsetPosition;
    private TargetHeight targetPosition;
    public double manualPower;

    public ElevatorMode elevatorMode;
    private DcMotorEx motorLeft, motorRight;
    private Robot robot;

    Elevator(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        this.robot = robot;

        motorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        //motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        downPosition = getRawEncoder();

        elevatorMode = ElevatorMode.DISABLED;
        targetPosition = TargetHeight.STONE_1;
    }

    public int getRawEncoder() {
        return motorRight.getCurrentPosition();
    }

    public int getEncoder() {
        lastEncoder = getRawEncoder() - downPosition;
        return lastEncoder;
    }

    public int getLastEncoder() {
        return lastEncoder;
    }

    public int getTargetEncoder() {
        if(elevatorMode == ElevatorMode.DOWN) {
            return 0;
        }
        return targetPosition.getEncoderPosition();
    }

    public int getDistanceLeft() {
        return getTargetEncoder() - lastEncoder + (int) offsetPosition;
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
            int distanceLeft = targetPosition.getEncoderPosition() - getEncoder() + (int) offsetPosition;
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
        targetPosition = targetPosition.next();
    }

    public void previousStone() {
        targetPosition = targetPosition.previous();
    }
}
