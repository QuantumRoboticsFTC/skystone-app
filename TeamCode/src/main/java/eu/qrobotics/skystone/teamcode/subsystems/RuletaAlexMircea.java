package eu.qrobotics.skystone.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RuletaAlexMircea implements Subsystem {

    public enum RuletaMode {
        OUT,
        IN,
        IDLE
    }

    public RuletaMode ruletaMode;
    Robot robot;
    CRServo ruleta;

    RuletaAlexMircea(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        ruleta = hardwareMap.get(CRServo.class, "ruleta");
        ruletaMode = RuletaMode.IDLE;
    }

    @Override
    public void update() {
        switch (ruletaMode) {
            case OUT:
                ruleta.setPower(-1);
                break;
            case IN:
                ruleta.setPower(0.5);
                break;
            case IDLE:
                ruleta.setPower(0);
                break;
        }
    }
}
