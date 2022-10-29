package org.firstinspires.ftc.teamcode.elevator;

import org.firstinspires.ftc.teamcode.auton_drvctrl.BotConfig;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous(group="PowerPlay_Elevator")
public class PP_Elevator_PIDTuner extends LinearOpMode {
    private String m_data_file_name = "HWTest_Elevator.txt";

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
    public void runOpMode() throws InterruptedException {
        // claw servo
//        Servo servo = hardwareMap.get(Servo.class, BotConfig.SERVO_CLAMP);
        //double currServoPos = 0;//servo.getPosition();

        // elevator motor
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, BotConfig.MOTOR_ELEVATOR);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
//        servo.setPosition(0); // initialize the servo osition

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidfOrig = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        motor.setVelocityPIDFCoefficients(ElevatorConstants.NEW_P, ElevatorConstants.NEW_I, ElevatorConstants.NEW_D, ElevatorConstants.NEW_F);
        motor.setPositionPIDFCoefficients(ElevatorConstants.NEW_P_POSITION);

        // re-read coefficients and verify change.
        PIDFCoefficients pidfModified = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double targetPos = 23;//16;//8;v //inch
        motor.setTargetPosition(ElevatorConstants.encoderInchesToTicks(targetPos));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(ElevatorConstants.MAX_VELOCITY * 1.0 );
        boolean dropped = false;
        while (opModeIsActive()) {
            telemetry.addData("velocity", motor.getVelocity());
            telemetry.addData("position", motor.getCurrentPosition());
            if (!dropped) {

    //            telemetry.addData("is at target", !motor.isBusy());

                // Loop while the motor is moving to the target
                while (motor.isBusy()) {
                    // Let the drive team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting for the motor to reach its target");
                    telemetry.update();
                }

                telemetry.addData("Runtime", "%.03f", getRuntime());

                // elevator reach the target position
                // now, turn the servo hand to drop the cube
                sleep(4000);
                telemetry.update();

                motor.setTargetPosition(ElevatorConstants.encoderInchesToTicks(0));

                while (motor.isBusy()) {
                    // Let the drive team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting for the motor to position 0");
                    telemetry.update();
                }

                motor.setVelocity(0);
//                servo.setPosition(0);
                sleep(1000);
                dropped = true;
            }
            else {
                break;
            }

        }
    }
}
