package eu.qrobotics.skystone.teamcode.cv.trackers;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvTracker;

import eu.qrobotics.skystone.teamcode.cv.filters.LeviColorFilter;

public class FixedStoneTracker extends OpenCvTracker {
    private Point pt1, pt2;
    private Mat mask;
    private double threshold;
    private StoneType stoneType;
    private LeviColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70);

    public FixedStoneTracker(Point leftUp, Point rightDown, double threshold) {
        pt1 = leftUp;
        pt2 = rightDown;
        this.threshold = threshold;
    }

    @Override
    public Mat processFrame(Mat input) {

        if (mask == null) {
            mask = new Mat(input.height(), input.width(), CvType.CV_8UC3);
        }

        mask.setTo(new Scalar(0, 0, 0));

        yellowFilter.process(input.clone(), mask);

        Imgproc.rectangle(input,
                pt1,
                pt2,
                new Scalar(255, 0, 0), 4);

        return mask;
    }

    public StoneType getStoneType() {
        return stoneType;
    }
}
