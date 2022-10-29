package org.firstinspires.ftc.teamcode.elevator;

import org.firstinspires.ftc.teamcode.auton_drvctrl.BotConfig;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group="PowerPlay_Elevator")
public class PP_Elevator_MaxVelocity extends LinearOpMode {

    /* Maximum Velocity Measurement - Elevator
     *
     * write down the max velocity after it's finished
     * */
    public static double RUN_TIME = 0.8; // in second
    private static ElapsedTime timer;


    private double maxV = 0; // in ticks/sec 2280;

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,BotConfig.MOTOR_ELEVATOR);
        /*
        * You can establish basic PIDF values for your mechanism by plugging a simple measurement into a
        formula. This will be sufficient for most use cases. The measurement you need is the maximum motor
        velocity of your mechanism. To get this, run a maximum velocity measurement Op Mode with a full
        battery. In this Op Mode, it is important that you set your motor’s mode to RUN_WITHOUT_ENCODER,
        and the power to 1.
        * */
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD); //?

        waitForStart();
        telemetry.clearAll();
        telemetry.update();

        motor.setPower(1);
        timer = new ElapsedTime();

        //while (!isStopRequested()
        while (!isStopRequested() && timer.seconds() < RUN_TIME) {
                double currentVelocity = motor.getVelocity(); //in ticks

                if (currentVelocity > maxV) {
                    maxV = currentVelocity;
                }
            }

        motor.setPower(0);

        telemetry.addData("Elevator Max Velocity (ticks/sec)", "%4.2f", maxV);
        telemetry.addData("max velocity (inch/sec)", "%4.2f", ElevatorConstants.encoderTicksToInches(maxV));
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) idle();

    }
}