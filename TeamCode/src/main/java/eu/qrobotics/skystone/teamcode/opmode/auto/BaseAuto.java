package eu.qrobotics.skystone.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;

public abstract class BaseAuto extends LinearOpMode {
    abstract Alliance getAlliance();
}
