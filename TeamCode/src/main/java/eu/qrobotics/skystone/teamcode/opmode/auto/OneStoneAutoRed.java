package eu.qrobotics.skystone.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;

@Autonomous
public class OneStoneAutoRed extends OneStoneAuto {
    @Override
    Alliance getAlliance() {
        return Alliance.RED;
    }
}
