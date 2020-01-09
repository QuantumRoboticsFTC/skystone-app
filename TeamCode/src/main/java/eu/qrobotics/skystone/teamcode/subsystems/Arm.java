package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm implements Subsystem {
    public static boolean IS_DISABLED = false;

    public enum GripperMode {
        CLOSE,
        OPEN,
        CAPSTONE
    }

    public static double GRIPPER_CLOSE_POSITION = 0.915;
    public static double GRIPPER_OPEN_POSITION = 0.4;
    public static double GRIPPER_CAPSTONE_POSITION = 0;

    public enum ArmMode {
        //INTAKE,
        IDLE,
        IDLE_AUTONOMY,
        OUTTAKE_HIGH,
        OUTTAKE_LOW
    }

    //public static double ARM_LEFT_INTAKE_POSITION = 0.089;
    //public static double ARM_RIGHT_INTAKE_POSITION = 0.911;
    public static double ARM_LEFT_IDLE_POSITION = 0;
    public static double ARM_RIGHT_IDLE_POSITION = 1;
    public static double ARM_LEFT_IDLE_AUTONOMY_POSITION = 0.097;
    public static double ARM_RIGHT_IDLE_AUTONOMY_POSITION = 0.903;
    public static double ARM_LEFT_OUTTAKE_HIGH_POSITION = 0.5;
    public static double ARM_RIGHT_OUTTAKE_HIGH_POSITION = 0.5;
    public static double ARM_LEFT_OUTTAKE_LOW_POSITION = 0.862;
    public static double ARM_RIGHT_OUTTAKE_LOW_POSITION = 0.138;

    public GripperMode gripperMode;
    public ArmMode armMode;
    
    private Servo leftArmServo, rightArmServo, gripperServo;
    private Robot robot;

    Arm(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        gripperMode = GripperMode.OPEN;
        armMode = ArmMode.IDLE;

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
    }

    @Override
    public void update() {
        if(IS_DISABLED)
            return;

        switch (gripperMode) {
            case OPEN:
                gripperServo.setPosition(GRIPPER_OPEN_POSITION);
                break;
            case CLOSE:
                gripperServo.setPosition(GRIPPER_CLOSE_POSITION);
                break;
            case CAPSTONE:
                gripperServo.setPosition(GRIPPER_CAPSTONE_POSITION);
                break;
        }

        switch (armMode) {
//            case INTAKE:
//                leftArmServo.setPosition(ARM_LEFT_INTAKE_POSITION);
//                rightArmServo.setPosition(ARM_RIGHT_INTAKE_POSITION);
//                break;
            case IDLE:
                leftArmServo.setPosition(ARM_LEFT_IDLE_POSITION);
                rightArmServo.setPosition(ARM_RIGHT_IDLE_POSITION);
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
