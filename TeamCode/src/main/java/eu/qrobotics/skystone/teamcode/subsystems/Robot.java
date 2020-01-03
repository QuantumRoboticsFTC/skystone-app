package eu.qrobotics.skystone.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

@Config
public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "Robot";

    private ExpansionHubEx hub1;
    private ExpansionHubEx hub2;
    private RevBulkData revBulkData1;
    private RevBulkData revBulkData2;
    private boolean isHub1Required = false;
    private boolean isHub2Required = false;

    public Drivetrain drive;
    public Intake intake;
    public Elevator elevator;
    public Arm arm;
    public FoundationGrabber foundationGrabber;
    public RuletaDeCacatALuiTudor ruleta;

    private List<Subsystem> subsystems;
    private List<Subsystem> subsystemsWithProblems;
    private ExecutorService subsystemUpdateExecutor;
    public FtcDashboard dashboard;
    public MovingStatistics top250, top100, top10;

    private boolean started;

    private static double getCurrentTime() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    private Runnable subsystemUpdateRunnable = () -> {
        double startTime, temp;
        while (!Thread.currentThread().isInterrupted()) {
            try {
                startTime = getCurrentTime(); // Get start time of update
                updateBulkData();
                revBulkData1 = hub1.getBulkInputData(); // Get data from hubs
                revBulkData2 = hub2.getBulkInputData();
                for (Subsystem subsystem : subsystems) { // Update all subsystems
                    if (subsystem == null) continue;
                    try {
                        subsystem.update();
                        subsystemsWithProblems.remove(subsystem);
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for " + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
                        Log.w(TAG, t);
                        if (!subsystemsWithProblems.contains(subsystem))
                            subsystemsWithProblems.add(subsystem);
                    }
                }
                temp = getCurrentTime() - startTime; // Calculate loop time
                top10.add(temp); // Add loop time to different statistics
                top100.add(temp);
                top250.add(temp);
            } catch (Throwable t) {
                Log.wtf(TAG, t); // If we get here, then something really weird happened.
            }
        }
    };

    public Robot(OpMode opMode, boolean isAutonomous) {
        // Initialize statistics
        top10 = new MovingStatistics(10);
        top100 = new MovingStatistics(100);
        top250 = new MovingStatistics(250);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        hub1 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        hub2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        revBulkData1 = hub1.getBulkInputData();
        revBulkData2 = hub2.getBulkInputData();

        //region Initialize subsystems
        subsystems = new ArrayList<>();

        try {
            arm = new Arm(opMode.hardwareMap, this);
            subsystems.add(arm);
        } catch (Exception e) {
            Log.w(TAG, "skipping Arm");
        }

        try {
            drive = new Drivetrain(opMode.hardwareMap, this, isAutonomous);
            subsystems.add(drive);
        } catch (Exception e) {
            Log.w(TAG, "skipping Drivetrain");
        }

        try {
            elevator = new Elevator(opMode.hardwareMap, this, isAutonomous);
            subsystems.add(elevator);
        } catch (Exception e) {
            Log.w(TAG, "skipping Elevator");
        }

        try {
            intake = new Intake(opMode.hardwareMap, this, isAutonomous);
            subsystems.add(intake);
        } catch (Exception e) {
            Log.w(TAG, "skipping Intake");
        }

        try {
            foundationGrabber = new FoundationGrabber(opMode.hardwareMap, this);
            subsystems.add(foundationGrabber);
        } catch (Exception e) {
            Log.w(TAG, "skipping FoundationGrabber");
        }

        try {
            ruleta = new RuletaDeCacatALuiTudor(opMode.hardwareMap, this);
            subsystems.add(ruleta);
        } catch (Exception e) {
            Log.w(TAG, "skipping RuletaDeCacatALuiTudor");
        }
        //endregion

        // Initialize update thread
        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
        subsystemsWithProblems = new ArrayList<>();
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            started = true;
        }
    }

    public void stop() {
        if (started && subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }
    }

    public void setIsHub1Required(boolean value) {
        isHub1Required = value;
    }

    public void setIsHub2Required(boolean value) {
        isHub2Required = value;
    }

    public ExpansionHubEx getHub1() {
        return hub1;
    }

    public ExpansionHubEx getHub2() {
        return hub2;
    }

    public RevBulkData getRevBulkDataHub1() {
        return revBulkData1;
    }

    public RevBulkData getRevBulkDataHub2() {
        return revBulkData2;
    }

    private void updateBulkData() {
        if (isHub1Required)
            revBulkData1 = hub1.getBulkInputData();
        if (isHub2Required)
            revBulkData2 = hub2.getBulkInputData();
    }

    public void sleep(double seconds) {
        try {
            Thread.sleep(Math.round(1000 * seconds));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    //region Automatically Stop Robot Loop
    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {

    }
    //endregion

    //region Global Warnings
    @Override
    public String getGlobalWarning() {
        List<String> warnings = new ArrayList<>();
        for (Subsystem subsystem : subsystemsWithProblems) {
            warnings.add("Problem with " + subsystem.getClass().getSimpleName());
        }
        return RobotLog.combineGlobalWarnings(warnings);
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void clearGlobalWarning() {
        subsystemsWithProblems.clear();
    }
    //endregion
}
