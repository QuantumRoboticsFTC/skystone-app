package eu.qrobotics.skystone.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RuletaDeCacatALuiTudor implements Subsystem {

    public enum RuletaMode {
        OUT,
        IDLE
    }

    public RuletaMode ruletaMode;
    Robot robot;
    CRServo ruleta;

    RuletaDeCacatALuiTudor(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        ruleta = hardwareMap.get(CRServo.class, "ruleta");
    }

    @Override
    public void update() {
        switch (ruletaMode) {
            case OUT:
                ruleta.setPower(-1);
                break;
            case IDLE:
                ruleta.setPower(0);
                break;
        }
    }
}
