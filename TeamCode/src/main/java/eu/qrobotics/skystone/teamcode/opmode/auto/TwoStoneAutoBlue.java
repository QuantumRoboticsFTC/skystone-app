package eu.qrobotics.skystone.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;

@Autonomous
public class TwoStoneAutoBlue extends TwoStoneAuto {
    @Override
    Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
