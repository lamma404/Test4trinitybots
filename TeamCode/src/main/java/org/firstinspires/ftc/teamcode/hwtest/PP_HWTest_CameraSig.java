package org.firstinspires.ftc.teamcode.hwtest;

import org.firstinspires.ftc.teamcode.auton_drvctrl.BotConfig;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SignalDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(group="PowerPlay_HWTest")
public class PP_HWTest_CameraSig extends LinearOpMode { // using RoadRunner
    private static final int  CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int  CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = BotConfig.WEBCAM_SIGNAL; // insert webcam name from configuration if using webcam

    public static double DISTANCE = 2; // in

    // store as variable here so we can access the location
    SignalDetector detector = new SignalDetector(CAMERA_WIDTH);
    OpenCvCamera camera;

    @Override
    public void runOpMode() {
        /*
        // Initialize the trajectories
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory trajectory_forward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectory_right = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        Trajectory trajectory_left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();
*/
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
            telemetry.addData("Signal number", detector.getSignalString());

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
