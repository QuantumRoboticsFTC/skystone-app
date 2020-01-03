/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package eu.qrobotics.skystone.teamcode.cv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.AutoTrajectoryGenerator;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.AutoTrajectoryGenerator.SkystonePattern;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils;
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;
import kotlin.Pair;

public class StoneDetector {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    private static final double[] skystonePositionsBlue = new double[]{-10, 30, 71};
    private static final double[] skystonePositionsRed = new double[]{300, 500, 800};

    private Alliance alliance;

    private static final int AVERAGE_COUNT = 500;

    private static final String VUFORIA_KEY =
            "AeS98kj/////AAABmXdQauRe0EckiIHBShZZ7Zwq2addwm/e1hNj2f5q5ApkyqGI7LhjE/c37puxUeyyCluEik9kbr+hEVPPL3iGlKbOaAy+6UmjaM7zXtsXEyoPjT9KdbAPlVbSbZ/FRUuQHa+fUCiNLOmTyD5EURAtR+hGV2q+IUvTt9jAeITP9NbhX0HWBa5l6DEp1noOvvug8VElcVjriGr6eiSdprFh7tPsPki+MO4oz8uveRCq5coebi3ET4BfRjYnmjx4xyaRlkh+P0xwpvNRkXuwH+lXjT1eyat+MywP2DEQBzjT+DZ7EncIT1qhQVKHfydX0ECTpR6jJsT70x6z8/Wg3V13UTug/bqVfloGMUEsILRbGJd5";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public StoneDetector(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        FtcDashboard.getInstance().startCameraStream(tfod, 0);
    }

    private List<Double> averageArray = new ArrayList<>();
    private List<Recognition> recognitions = new ArrayList<>();
    public void update() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                recognitions = updatedRecognitions;
            }
            double position = 100000;
            for (Recognition recognition : recognitions) {
                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                    if (alliance == Alliance.BLUE)
                        position = Math.min(position, (double) recognition.getLeft());
                    else
                        position = Math.min(position, (double) recognition.getRight());
                }
            }
            if (position > 5000) {
                if (alliance == Alliance.BLUE) {
                    position = -10;
                } else {
                    position = 800;
                }
            }
            averageArray.add(position);
            if (averageArray.size() > AVERAGE_COUNT)
                averageArray.remove(0);
        }
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public Pair<SkystonePattern, Double> getSkystone() {
        if (averageArray.size() != 0) {
            double averagePosition = 0;
            for (Double pos :
                    averageArray) {
                averagePosition += pos;
            }
            averagePosition /= averageArray.size();
            int minPos = 0;
            double min = 10000;
            for (int i = 0; i < 3; i++) {
                if (alliance == Alliance.RED) {
                    if (Math.abs(averagePosition - skystonePositionsRed[i]) < min) {
                        min = Math.abs(averagePosition - skystonePositionsRed[i]);
                        minPos = i;
                    }
                } else {
                    if (Math.abs(averagePosition - skystonePositionsBlue[i]) < min) {
                        min = Math.abs(averagePosition - skystonePositionsBlue[i]);
                        minPos = i;
                    }
                }
            }
            SkystonePattern skystone;
            if (minPos == 0)
                skystone = SkystonePattern.LEFT;
            else if (minPos == 1)
                skystone = SkystonePattern.MIDDLE;
            else
                skystone = SkystonePattern.RIGHT;
            return new Pair<>(skystone, averagePosition);
        }
        return new Pair<>(SkystonePattern.LEFT, 100000.0);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.85;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}