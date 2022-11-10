package org.firstinspires.ftc.teamcode.auton_drvctrl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RR_MecanumDrive;
import org.firstinspires.ftc.teamcode.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/* == Autonomous option 1 ==
 * [Name on driver station]
 *
 * [Functions] - within 30 seconds
 * 1. Use the camera to detect the signals on the self-made sleeve
 * 2. Park the bot at the Signal Zone
 * ... depending on the functionality and efficiency of the elevator & clamp structure, we may add a step to put a cone on a junction before parking.
 *
 * * */
@Autonomous(group="PowerPlay_Auton")
public class PP_Auton_Opt1_L extends LinearOpMode {
    // drivetrain
    private RR_MecanumDrive m_drivetrain = null;

    /* elevator */
    private DcMotorEx m_elevatorMotor = null;


    // clamp
    private Servo m_clampServo = null;

    /* start of camera related setting */
    private OpenCvCamera m_camera = null;
    private AprilTagDetector m_aprilTagDetector;
    private AprilTagDetection m_signalTag = null;

    private static final int  CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int  CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final double FEET_PER_METER = 3.28084; //???

    // Lens intrinsics
    /* C270 webcam at 320 x 240 */
    double fx = 231.2;
    double fy = 231.2;
    double cx = 160.8;
    double cy = 118.392;

    // UNITS ARE METERS
    double tagsize = 0.35;

    // Tag ID 11,12,13 from the 36h11 family
    int LEFT = 11;
    int MIDDLE = 12;
    int RIGHT = 13;

    /* end of camera related setting */

    @Override
    public void runOpMode() {
        m_drivetrain = new RR_MecanumDrive(hardwareMap);
        m_drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* initialize clamp, close with a preload cone  */
        m_clampServo = hardwareMap.get(Servo.class, BotConfig.SERVO_CLAMP);
        m_clampServo.setPosition(BotConfig.SERVO_CLOSE_POS);

        /* initlizate elevator */
        m_elevatorMotor = hardwareMap.get(DcMotorEx.class,BotConfig.MOTOR_ELEVATOR);
        m_elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        m_elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidfOrig = m_elevatorMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        m_elevatorMotor.setVelocityPIDFCoefficients(ElevatorConstants.NEW_P, ElevatorConstants.NEW_I, ElevatorConstants.NEW_D, ElevatorConstants.NEW_F);
        m_elevatorMotor.setPositionPIDFCoefficients(ElevatorConstants.NEW_P_POSITION);

        // re-read coefficients and verify change.
        PIDFCoefficients pidfModified = m_elevatorMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        // Initialize the trajectories
        Trajectory trajectory_groundJunction = m_drivetrain.trajectoryBuilder(new Pose2d())
                .back(13.5)
                .build();
        Trajectory trajectory_groundJunctiontask2 = m_drivetrain.trajectoryBuilder(trajectory_groundJunction.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(4)
                .build();
        Trajectory trajectory_groundJunctiontask3 = m_drivetrain.trajectoryBuilder(trajectory_groundJunctiontask2.end())
                .back(5)
                .build();

        Trajectory trajectory_park = m_drivetrain.trajectoryBuilder(trajectory_groundJunctiontask3.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(10)
                .build();
        Trajectory trajectory_parkLeft = m_drivetrain.trajectoryBuilder(trajectory_park.end())
                .strafeLeft(20)
                .build();
        Trajectory trajectory_parkRight = m_drivetrain.trajectoryBuilder(trajectory_park.end())
                .strafeRight(20)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        m_camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam_signal"), cameraMonitorViewId);
        m_aprilTagDetector = new AprilTagDetector(tagsize, fx, fy, cx, cy);

        m_camera.setPipeline(m_aprilTagDetector);
        m_camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                m_camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) { // detect signal on the sleeve
            detectSignal();
            boolean tagFound = false;

            if ((m_signalTag != null) &&
                    (m_signalTag.id == LEFT || m_signalTag.id == MIDDLE || m_signalTag.id == RIGHT)) {
                tagFound = true;
            }

            if (tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(m_signalTag);

            } else {
                telemetry.addLine("Don't see tag of interest!!!");
            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(m_signalTag == null)
        {
            // didn't detect the signal at init, now detect again
            detectSignal(); //?
            telemetry.addLine("Detecting Tag:\n");
        }
        telemetry.addLine("Tag detected:\n");
        tagToTelemetry(m_signalTag);
        telemetry.update();

        /* Actually do something useful */
        // firstly put the preload cone onto the ground junction
        m_drivetrain.followTrajectory(trajectory_groundJunction);
        m_drivetrain.turn(Math.toRadians(-90));
        m_drivetrain.followTrajectory(trajectory_groundJunctiontask2);
        sleep(50);
        m_clampServo.setPosition(BotConfig.SERVO_OPEN_POS); //release the cone
        sleep(300);

        m_drivetrain.followTrajectory(trajectory_groundJunctiontask3);
        m_drivetrain.turn(Math.toRadians(-90));

        // now park at the signal area
        m_drivetrain.followTrajectory(trajectory_park);

        if(m_signalTag == null || m_signalTag.id == LEFT){
            m_drivetrain.followTrajectory(trajectory_parkLeft);
        }else if(m_signalTag.id == MIDDLE){
            //trajectory
        }else{
            //trajectory
            m_drivetrain.followTrajectory(trajectory_parkRight);
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
        telemetry.addLine("Autonomous is completed.");
        telemetry.update();
    }

    void detectSignal(){
        ArrayList<AprilTagDetection> currentDetections = m_aprilTagDetector.getLatestDetections();
        if(currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                {
                    m_signalTag = tag;
                    break;
                }
            }
        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("\nTranslation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
