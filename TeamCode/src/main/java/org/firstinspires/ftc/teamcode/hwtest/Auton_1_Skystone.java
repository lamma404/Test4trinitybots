package org.firstinspires.ftc.teamcode.hwtest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RR_MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="PowerPlay_HWTest_Camera_ss", group="PowerPlay_HWTest")
public class Auton_1_Skystone extends LinearOpMode { // using RoadRunner

    private static final int  CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int  CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam_front"; // insert webcam name from configuration if using webcam

    public static double DISTANCE = 2; // in

    // store as variable here so we can access the location
    SkystoneDetector detector = new SkystoneDetector(CAMERA_WIDTH);
    OpenCvCamera camera;

    @Override
    public void runOpMode() {

        RR_MecanumDrive drive = new RR_MecanumDrive(hardwareMap);

        Trajectory trajectory_forward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectory_right = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        Trajectory trajectory_left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();

        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        }
        else { // phone internal camera
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(detector);
//        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        // Connect to the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Skystone position", detector.getLocation());

//
//            if (detector.getLocation().equals("Left")){
//                drive.followTrajectory(trajectory_left);
//                telemetry.addData("drive direction", "Left");
//            }else if (detector.getLocation().equals("Right")){
//                drive.followTrajectory(trajectory_right);
//                telemetry.addData("drive direction", "Right");
//            }

            telemetry.update();
            sleep(1000);
        }
    }
}
