package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetector extends OpenCvPipeline {
    enum SkystoneLocation {
        LEFT,
        RIGHT,
        NONE
    }

    private int width = 0; // width of the image
    private int height = 0;

    SkystoneLocation location;
    private Telemetry telemetry = null;

    public SkystoneDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * @param width The width of the image (check your camera)
     */
    public SkystoneDetector(int width) {
        this.width = width;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (width == 0) width = input.width();
        if (height == 0) height = input.height();

        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {
            location = SkystoneLocation.NONE;
            return input;
        }

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(14, 59, 70); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(76, 255, 255); // higher bound HSV for yellow

        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
//        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        boolean left = false; // true if regular stone found on the left side
        boolean right = false; // "" "" on the right side

/* original code from github:
 https://gist.github.com/oakrc/12a7b5223df0cb55d7c1288ce96a6ab7

for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x)
                left = true;
            if (boundRect[i].x + boundRect[i].width > right_x)
                right = true;

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }
*/

//        telemetry.addData("left_x", "%4.2f",left_x);
//        telemetry.addData("right_x", "%4.2f",right_x);
//        telemetry.addData("boundRect.length", "%d", boundRect.length);
        for (int i = 0; i != boundRect.length; i++) {
//            telemetry.addData("i", "%d", i);
//            telemetry.addData("boundRect(x,y)", "(%d, %d)", boundRect[i].x, boundRect[i].y);
//            telemetry.addData("boundRect(width, height)", "(%d, %d)", boundRect[i].width, boundRect[i].height);

            if ((boundRect[i].width >= width * 0.1) && (boundRect[i].height >= height * 0.1))
            {// && (boundRect[i].width > boundRect[i].height)) {
                // draw red bounding rectangles on mat
                // the mat has been converted to HSV so we need to use HSV as well
//                telemetry.addData("boundRect(x,y)", "(%d, %d)", boundRect[i].x, boundRect[i].y);
//                telemetry.addData("boundRect(width, height)", "(%d, %d)", boundRect[i].width, boundRect[i].height);
                Imgproc.rectangle(mat, boundRect[i], new Scalar(120, 100, 100), 2);
                if (boundRect[i].x < left_x) {
                    left = true;
//                    telemetry.addData("left changed to", "true");
                }
                if (boundRect[i].x + boundRect[i].width > right_x) {
                    right = true;
//                    telemetry.addData("right changed to", "true");
                }
            }
        }

        // if there is no yellow regions on a side
        // that side should be a Skystone
        if (!left) location = SkystoneLocation.LEFT;
        else if (!right) location = SkystoneLocation.RIGHT;
            // if both are true, then there's no Skystone in front.
            // since our team's camera can only detect two at a time
            // we will need to scan the next 2 stones
        else location = SkystoneLocation.NONE;

        /**
         * Add some nice and informative telemetry messages
         */
//        telemetry.addData("[>]","SkystoneDector Result ==>");
//        if (location == SkystoneLocation.LEFT)
//            telemetry.addData("Skystone location", "Left");
//        else if (location == SkystoneLocation.RIGHT)
//            telemetry.addData("Skystone location", "Right");
//        else telemetry.addData("Skystone location", "None");
//        telemetry.update();

        return mat; // return the mat with rectangles drawn
    }

//    public SkystoneLocation getLocation() {
//        return this.location;
//    }

    public String getLocation() {
        if (this.location == SkystoneLocation.LEFT) return "Left";
        if (this.location == SkystoneLocation.RIGHT) return "Right";
        if (this.location == SkystoneLocation.NONE) return "None";
        return "";

    }
}
