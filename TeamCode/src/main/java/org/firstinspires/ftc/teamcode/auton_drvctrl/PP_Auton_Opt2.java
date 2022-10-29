package org.firstinspires.ftc.teamcode.auton_drvctrl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RR_MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SignalDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/* == Autonomous option 1 ==
* [Name on driver station]  PowerPlay:Auton_Opt1
*
* [Functions] - within 30 seconds
* 1. Use the camera to detect the signals on the self-made sleeve
* 2. Park the bot at the Signal Zone
* ... depending on the functionality and efficiency of the elevator & clamp structure, we may add a step to put a cone on a junction before parking.
*
* * */
@Autonomous(group="PowerPlay_Auton")
public class PP_Auton_Opt2 extends LinearOpMode {
    RR_MecanumDrive m_drivetrain = null;

    private static final int  CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int  CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    public static double DISTANCE = 2; // in

    // store as variable here so we can access the location
    SignalDetector detector = new SignalDetector(CAMERA_WIDTH);
    OpenCvCamera camera;

    @Override
    public void runOpMode() {
        m_drivetrain = new RR_MecanumDrive(hardwareMap);
        m_drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Initialize the trajectories
        Trajectory trajectory_forward = m_drivetrain.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();
/*
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
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, BotConfig.WEBCAM_SIGNAL), cameraMonitorViewId);
        }
        else { // phone internal camera
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(detector);

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

        if (isStopRequested()) return;

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

        m_drivetrain.followTrajectory(trajectory_forward);

        m_drivetrain.setDrivePower(new Pose2d(0.0, 0.0, 0.0));
        if (m_drivetrain != null) {
            m_drivetrain = null;
        }
        while (!isStopRequested() && opModeIsActive()) ;

    }

}
