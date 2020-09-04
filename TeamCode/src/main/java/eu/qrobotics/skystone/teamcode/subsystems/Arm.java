package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm implements Subsystem {
    public static boolean IS_DISABLED = false;

    public enum GripperMode {
        INTAKE,
        GRIP,
        DROP,
        CAPSTONE
    }

    public static double GRIPPER_FRONT_INTAKE_POSITION = 1;
    public static double GRIPPER_BACK_INTAKE_POSITION = 1;

    public static double GRIPPER_FRONT_GRIP_POSITION = 0.1;
    public static double GRIPPER_BACK_GRIP_POSITION = 1;

    public static double GRIPPER_FRONT_DROP_POSITION = 0.2;
    public static double GRIPPER_BACK_DROP_POSITION = 0.54;

    public static double GRIPPER_FRONT_CAPSTONE_POSITION = 0.2;
    public static double GRIPPER_BACK_CAPSTONE_POSITION = 0;

    public enum ArmMode {
        FRONT,
        AUTONOMOUS_BACK, // why...
        BACK
    }

    public static double ARM_FRONT_POSITION = 0;
    public static double ARM_AUTONOMOUS_BACK_POSITION = 0.95;
    public static double ARM_BACK_POSITION = 1;

    public GripperMode gripperMode;
    public ArmMode armMode;

    private Servo armServo, gripperFrontServo, gripperBackServo;
    private Robot robot;

    Arm(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        gripperMode = GripperMode.INTAKE;
        armMode = ArmMode.FRONT;

        armServo = hardwareMap.get(Servo.class, "slider");
        gripperFrontServo = hardwareMap.get(Servo.class, "gripperFront");
        gripperBackServo = hardwareMap.get(Servo.class, "gripperBack");
    }

    @Override
    public void update() {
        if (IS_DISABLED)
            return;

        switch (gripperMode) {
            case INTAKE:
                gripperFrontServo.setPosition(GRIPPER_FRONT_INTAKE_POSITION);
                gripperBackServo.setPosition(GRIPPER_BACK_INTAKE_POSITION);
                break;
            case GRIP:
                gripperFrontServo.setPosition(GRIPPER_FRONT_GRIP_POSITION);
                gripperBackServo.setPosition(GRIPPER_BACK_GRIP_POSITION);
                break;
            case DROP:
                gripperFrontServo.setPosition(GRIPPER_FRONT_DROP_POSITION);
                gripperBackServo.setPosition(GRIPPER_BACK_DROP_POSITION);
                break;
            case CAPSTONE:
                gripperFrontServo.setPosition(GRIPPER_FRONT_CAPSTONE_POSITION);
                gripperBackServo.setPosition(GRIPPER_BACK_CAPSTONE_POSITION);
                break;
        }
        switch (armMode) {
            case FRONT:
                armServo.setPosition(ARM_FRONT_POSITION);
                break;
            case AUTONOMOUS_BACK:
                armServo.setPosition(ARM_AUTONOMOUS_BACK_POSITION);
                break;
            case BACK:
                armServo.setPosition(ARM_BACK_POSITION);
                break;
        }
    }
}
