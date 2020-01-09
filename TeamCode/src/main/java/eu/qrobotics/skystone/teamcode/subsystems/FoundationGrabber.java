package eu.qrobotics.skystone.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGrabber implements Subsystem {

    public enum FoundationGrabberMode {
        UP,
        DOWN
    }

    private Servo leftFoundationGrabber;
    private Servo rightFoundationGrabber;

    private Robot robot;
    public FoundationGrabberMode foundationGrabberMode;

    FoundationGrabber(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        leftFoundationGrabber = hardwareMap.get(Servo.class, "leftFoundationServo");
        rightFoundationGrabber = hardwareMap.get(Servo.class, "rightFoundationServo");

        foundationGrabberMode = FoundationGrabberMode.UP;
    }

    @Override
    public void update() {
        switch (foundationGrabberMode) {
            case UP:
                leftFoundationGrabber.setPosition(0.075);
                rightFoundationGrabber.setPosition(0.725);
                break;
            case DOWN:
                leftFoundationGrabber.setPosition(0.675);
                rightFoundationGrabber.setPosition(0.115);
                break;
        }
    }
}
