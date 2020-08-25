package eu.qrobotics.skystone.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;

@Autonomous
public class TwoStoneAutoRed extends TwoStoneAuto {
    @Override
    Alliance getAlliance() {
        return Alliance.RED;
    }
}
