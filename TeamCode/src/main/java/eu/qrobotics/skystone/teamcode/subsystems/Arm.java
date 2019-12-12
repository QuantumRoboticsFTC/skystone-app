package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm implements Subsystem {
    public enum GripperMode {
        CLOSE,
        OPEN
    }

    public static double GRIPPER_CLOSE_POSITION = 1;
    public static double GRIPPER_OPEN_POSITION = 0;

    public enum ArmMode {
        INTAKE,
        START,
        OUTTAKE_HIGH,
        OUTTAKE_LOW
    }

    public static double ARM_LEFT_INTAKE_POSITION = 1;
    public static double ARM_RIGHT_INTAKE_POSITION = 0;
    public static double ARM_LEFT_START_POSITION = 1;
    public static double ARM_RIGHT_START_POSITION = 0;
    public static double ARM_LEFT_OUTTAKE_HIGH_POSITION = 1;
    public static double ARM_RIGHT_OUTTAKE_HIGH_POSITION = 0;
    public static double ARM_LEFT_OUTTAKE_LOW_POSITION = 1;
    public static double ARM_RIGHT_OUTTAKE_LOW_POSITION = 0;

    private GripperMode gripperMode;
    private ArmMode armMode;
    
    private Servo leftArmServo, rightArmServo, gripperServo;
    private Robot robot;

    Arm(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        gripperMode = GripperMode.OPEN;
        armMode = ArmMode.INTAKE;

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
    }

    @Override
    public void update() {
        switch (gripperMode) {
            case OPEN:
                gripperServo.setPosition(GRIPPER_OPEN_POSITION);
                break;
            case CLOSE:
                gripperServo.setPosition(GRIPPER_CLOSE_POSITION);
                break;
        }

        switch (armMode) {
            case INTAKE:
                leftArmServo.setPosition(ARM_LEFT_INTAKE_POSITION);
                rightArmServo.setPosition(ARM_RIGHT_INTAKE_POSITION);
                break;
            case START:
                leftArmServo.setPosition(ARM_LEFT_START_POSITION);
                rightArmServo.setPosition(ARM_RIGHT_START_POSITION);
                break;
            case OUTTAKE_HIGH:
                leftArmServo.setPosition(ARM_LEFT_OUTTAKE_HIGH_POSITION);
                rightArmServo.setPosition(ARM_RIGHT_OUTTAKE_HIGH_POSITION);
                break;
            case OUTTAKE_LOW:
                leftArmServo.setPosition(ARM_LEFT_OUTTAKE_LOW_POSITION);
                rightArmServo.setPosition(ARM_RIGHT_OUTTAKE_LOW_POSITION);
                break;
        }
    }
}
