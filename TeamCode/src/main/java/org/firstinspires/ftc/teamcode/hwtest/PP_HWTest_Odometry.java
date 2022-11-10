package org.firstinspires.ftc.teamcode.hwtest;

import org.firstinspires.ftc.teamcode.auton_drvctrl.BotConfig;
import android.os.Environment;

import com.qualcomm.hardware.lynx.LynxModule;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
//import com.qualcomm.robotcore.hardware.DcMotor$ZeroPowerBehavior;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This program is to test the drive train that's built with Gobilda's Mecanum (X-drive) chassis
 *
 * Everything in this test is auto-run, no need to use the gamepad
 *
 **/

@TeleOp(group="PowerPlay_HWTest")

public class PP_HWTest_Odometry extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private String m_test_program_name = "Odometry";
    private String m_data_file_name = "HWTest_" + m_test_program_name + ".txt";

    public String appendToDataFile(StringBuffer strToWrite){
        String rv = "complete";

        if (strToWrite.length() > 0) {
            String fPath = String.format("%s/FIRST/data",
                    Environment.getExternalStorageDirectory().getAbsolutePath());

            File directory = new File(fPath);
            if (!directory.exists())
            {
                directory.mkdirs();
            }

            //String fPath = String.format("/sdcard/FIRST/%s", m_data_file_name);
            String fn = fPath + "/" + m_data_file_name;
            File f = new File(fn);
            if (f.exists())
            {
                f.delete();
                rv = fn + " - old file was deleted.\n";
            }
            try(FileWriter fWriter = new FileWriter(fn, false)){
                fWriter.write(strToWrite.toString());
            } catch (IOException e) {
                rv = e.getMessage();
            }
            rv += "file completed";
        }
        return (rv);
    }
    private static double encoderTicksToInches(double ticks) {
        double TICKS_PER_REV = 8192; // Rev Encoder
        double WHEEL_RADIUS = 0.6889; // in, 35 mm Diameter
        double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    @Override
    public void runOpMode() {
        double max_power = 0.4; // slow speed - voltage
        double MAX_TIME = 3.0; // time ellapse

        StringBuffer logString = new StringBuffer();
        logString.setLength(0);
        logString.append("# " + m_test_program_name + "\n");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, BotConfig.MOTOR_LEFT_FRONT);
        leftBackDrive  = hardwareMap.get(DcMotor.class,  BotConfig.MOTOR_LEFT_BACK);
        rightFrontDrive = hardwareMap.get(DcMotor.class, BotConfig.MOTOR_RIGHT_FRONT);
        rightBackDrive = hardwareMap.get(DcMotor.class, BotConfig.MOTOR_RIGHT_BACK);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(BotConfig.MOTOR_LEFT_FRONT_DIR);
        leftBackDrive.setDirection(BotConfig.MOTOR_LEFT_BACK_DIR);
        rightFrontDrive.setDirection(BotConfig.MOTOR_RIGHT_FRONT_DIR);
        rightBackDrive.setDirection(BotConfig.MOTOR_RIGHT_BACK_DIR);

//        public static final DcMotorSimple.Direction MOTOR_LEFT_BACK_DIR = DcMotorSimple.Direction.REVERSE;
//        public static final DcMotorSimple.Direction MOTOR_RIGHT_BACK_DIR = DcMotorSimple.Direction.FORWARD;
//        public static final DcMotorSimple.Direction MOTOR_LEFT_FRONT_DIR = DcMotorSimple.Direction.REVERSE;
//        public static final DcMotorSimple.Direction MOTOR_RIGHT_FRONT_DIR = DcMotorSimple.Direction.FORWARD;

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (LynxModule hub : allHubs)
        {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double final_ticks;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())  {

            if (runtime.seconds() <= MAX_TIME) {
                // Send calculated power to wheels
                leftFrontDrive.setPower(max_power);
                rightFrontDrive.setPower(max_power);
                leftBackDrive.setPower(max_power);
                rightBackDrive.setPower(max_power);

                int leftFrontEncoderPos = -1 * leftFrontDrive.getCurrentPosition();
                int rightFrontEncoderPos = -1 * rightFrontDrive.getCurrentPosition();
                int leftBackEncoderPos = leftBackDrive.getCurrentPosition();
                int rightBackEncoderPos = rightBackDrive.getCurrentPosition();

                // Show the elapsed game time and wheel power.
                String tmpStr = String.format("%s (LF_2,RF_3,LB_0,RB_1)=%d, %d, %d, %d\n", runtime.toString(),
                        leftFrontEncoderPos, rightFrontEncoderPos, leftBackEncoderPos, rightBackEncoderPos);
                logString.append(tmpStr);

                final_ticks = (leftFrontEncoderPos+ rightFrontEncoderPos) /2;

                logString.append(String.format("final_distance=%4.2f\n", encoderTicksToInches(final_ticks)));

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("Front left/Right", "%d, %d", leftFrontEncoderPos, rightFrontEncoderPos);
                telemetry.addData("Back  left/Right", "%d, %d", leftBackEncoderPos, rightBackEncoderPos);
                telemetry.update();
                sleep(20); // we don't need to check all the time
            }
           /* else if (runtime.seconds() <= MAX_TIME * 1.5)
            {
                // circle counter clockwise
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(max_power*0.8);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(max_power*0.8);

                int leftFrontEncoderPos = leftFrontDrive.getCurrentPosition();
                int rightFrontEncoderPos = rightFrontDrive.getCurrentPosition();
                int leftBackEncoderPos = leftBackDrive.getCurrentPosition();
                int rightBackEncoderPos = rightBackDrive.getCurrentPosition();

                // Show the elapsed game time and wheel power.
                String tmpStr = String.format("%s (LF,RF,LB,RB)=%d, %d, %d, %d\n" , runtime.toString(),
                        leftFrontEncoderPos, rightFrontEncoderPos, leftBackEncoderPos, rightBackEncoderPos);
                logString.append(tmpStr);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("Front left/Right", "%d, %d", leftFrontEncoderPos, rightFrontEncoderPos);
                telemetry.addData("Back  left/Right", "%d, %d", leftBackEncoderPos, rightBackEncoderPos);
                telemetry.update();
                sleep(20); // we don't need to check all the time
            }*/
            else {

                String rv = appendToDataFile(logString);
                telemetry.addData("Write file status:", logString);
                telemetry.update();

                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);

                sleep(5000); // allow to read telemetry
                break;
            }

        }
    }


}

