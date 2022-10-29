package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SignalDetector extends OpenCvPipeline {
    enum SignalNumber {
        SIG_1_LEFT,   //Square
        SIG_2_MIDDLE, //Circle
        SIG_3_RIGHT   //Rectangle
    }
    private int width = 0; // width of the image
    private int height = 0;

    // We create a HSV range for yellow to detect the shapes
    // NOTE: In OpenCV's implementation,
    // Hue values are half the real value
    private static Scalar lowHSV = new Scalar(14, 59, 70); // lower bound HSV for yellow
    private static Scalar highHSV = new Scalar(76, 255, 255); // higher bound HSV for yellow
    private static Scalar redScalarHSV = new Scalar(240, 100, 100); // red color in HSV - for drawing the detected shape

    private double signal_check_window_size = 0;
    private double signal_top_left_x = 0;
    private double signal_top_left_y = 0;
    private double square_size_min = 0;

    private static double scale_signal_check_window_size = 0.2;
    private static double scale_signal_top_left_x = 0.4;
    private static double scale_signal_top_left_y = 0.38;
    private static double scale_square_size_min = .075;

    private Mat m_mat = null;
    private SignalNumber signalNumber;

    private Telemetry telemetry = null;

    public SignalDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * @param width The width of the image (check your camera)
     */
    public SignalDetector(int width) {
        this.width = width;
    }
    private boolean isCircleSignal(Mat input) {
        boolean ret = false;

        Mat gray = new Mat();
        Mat circles = new Mat();
        Point center = null;

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.medianBlur(gray, gray, 5);
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                (double)gray.rows()/16, // change this value to detect circles with different distances to each other
                100.0, 30.0, 10, 30); // change the last two parameters (min_radius & max_radius) to detect larger circles

        //telemetry.addData("circles.cols()=", circles.cols());

        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            center = new Point(Math.round(c[0]), Math.round(c[1]));
            // circle center
//            Imgproc.circle(mat, center, 1, new Scalar(0,100,100), 3, 8, 0 );
            // circle outline
            int radius = (int) Math.round(c[2]);
            Imgproc.circle(m_mat, center, radius, new Scalar(255,0,255), 2, 8, 0 );

            ret = true;
            break;
        }
        if (gray != null) {
            gray.release();
            gray = null;
        }
        if (circles != null) {
            circles.release();
            circles = null;
        }
        //if (center != null) center = null;

        return(ret);
    }
    @Override
    public Mat processFrame(Mat input) {
        // get the size range for the image check
        if (width == 0) width = input.width();
        if (height == 0) height = input.height();
        signal_check_window_size = scale_signal_check_window_size * width;
        signal_top_left_x = scale_signal_top_left_x * width;
        signal_top_left_y = scale_signal_top_left_y * height;
        square_size_min = scale_square_size_min * width;

        // all the temporary shape related variables
        Mat edges = null;
        Mat thresh = null;
        Mat hierarchy = null;
        MatOfPoint2f[] contoursPoly = null;
        Rect[] boundRect = null;
        List<MatOfPoint> contours = null;

        // Make a working copy of the input matrix in HSV
        if (m_mat != null) {
            m_mat.release();
            m_mat = null; // make sure the memory for the previous matrix is released
        }

        m_mat = new Mat();
        Imgproc.cvtColor(input, m_mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume signal is 2
        if (m_mat.empty()) {
            signalNumber = SignalNumber.SIG_2_MIDDLE;
            return input;
        }

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        thresh = new Mat();
        Core.inRange(m_mat, lowHSV, highHSV, thresh);

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
//        telemetry.addData("signal_top_left(x,y)=", "(%4.2f, %4.2f)", signal_top_left_x, signal_top_left_y);
//        telemetry.addData("signal_bottom_right(x,y)=", "(%4.2f, %4.2f)", (signal_top_left_x +signal_check_window_size) , (signal_top_left_y+signal_check_window_size));

        // detect circle ==> 2
        if (isCircleSignal(input)){
            signalNumber = SignalNumber.SIG_2_MIDDLE;
        }
        else{
            // detect square or rectangle ==> 1 or 3

            // Use Canny Edge Detection to find edges, you might have to tune the thresholds for hysteresis
            edges = new Mat();
            Imgproc.Canny(thresh, edges, 100, 300);

            // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
            // Oftentimes the edges are disconnected. findContours connects these edges.
            // We then find the bounding rectangles/circle/triangle of those contours
            contours = new ArrayList<>();
            hierarchy = new Mat();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            boundRect = new Rect[contours.size()];
            contoursPoly = new MatOfPoint2f[contours.size()];

            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }

            // ok, we've got the contours. let's check the shape
            for (int i = 0; i != boundRect.length; i++) {
    //            telemetry.addData("i", "%d", i);
    //            telemetry.addData("boundRect(x,y)", "(%d, %d)", boundRect[i].x, boundRect[i].y);
    //            telemetry.addData("boundRect(width, height)", "(%d, %d)", boundRect[i].width, boundRect[i].height);

                // draw red bounding square on mat
                // the mat has been converted to HSV so we need to use HSV as well
                if (boundRect[i].x >= signal_top_left_x && boundRect[i].x <= (signal_top_left_x + signal_check_window_size) &&
                        boundRect[i].y >= signal_top_left_y && boundRect[i].y <= (signal_top_left_y + signal_check_window_size) &&
                        boundRect[i].width >= square_size_min){
                    // found a candidate

                    // square : width = height
                    if (Math.abs(boundRect[i].width - boundRect[i].height) < 0.1 * signal_check_window_size) {
                        Imgproc.rectangle(m_mat, boundRect[i], redScalarHSV, 2);
                        if (telemetry != null) {
                            telemetry.addData("boundRect(x,y)", "(%d, %d)", boundRect[i].x, boundRect[i].y);
                            telemetry.addData("boundRect(width, height)", "(%d, %d)", boundRect[i].width, boundRect[i].height);
                            telemetry.addData("==> ", "found a square");
                        }
                        signalNumber = SignalNumber.SIG_1_LEFT;
                        break;
                    }
                    else if (Math.abs(boundRect[i].width - boundRect[i].height * 2) < 0.1 * signal_check_window_size ) {
                        // rectangle: width = height * 2
                        Imgproc.rectangle(m_mat, boundRect[i], redScalarHSV, 2);
                        if (telemetry != null) {
                            telemetry.addData("boundRect(x,y)", "(%d, %d)", boundRect[i].x, boundRect[i].y);
                            telemetry.addData("boundRect(width, height)", "(%d, %d)", boundRect[i].width, boundRect[i].height);
                            telemetry.addData("==> ", "found a rectangle");
                        }
                        signalNumber = SignalNumber.SIG_3_RIGHT;
                        break;
                    }
                }
            }
        }

        if (telemetry != null) {
            telemetry.addData("Signal number", signalNumber);
        }
        if (edges != null) {
            edges.release();
            edges = null;
        }
        // if (boundRect != null) boundRect = null;
        if (contours != null) {
            contours = null;
        }
        if (contoursPoly != null) contoursPoly = null;
        if (hierarchy != null) {
            hierarchy.release();
            hierarchy = null;
        }
        if (thresh != null) {
            thresh.release();
            thresh = null;
        }

        return m_mat;
    }

    public SignalNumber getSignalNumber() {
        return this.signalNumber;
    }
    public String getSignalString() {
        if (this.signalNumber == SignalNumber.SIG_1_LEFT) return "SIG_1_LEFT";
        else if (this.signalNumber == SignalNumber.SIG_2_MIDDLE) return "SIG_2_MIDDLE";
        else if (this.signalNumber == SignalNumber.SIG_3_RIGHT) return "SIG_3_RIGHT";
        else return ("");
    }
}
