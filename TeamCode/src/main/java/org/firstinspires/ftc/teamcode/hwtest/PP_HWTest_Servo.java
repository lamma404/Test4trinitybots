
package org.firstinspires.ftc.teamcode.hwtest;

import org.firstinspires.ftc.teamcode.auton_drvctrl.BotConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group="PowerPlay_HWTest")
public class PP_HWTest_Servo extends OpMode {
    private Servo testServo;
    private double position;
    private double START_POS = .6;
    private double CLOSE_POS = 1;

    public void init(){
        testServo = hardwareMap.get(Servo.class, BotConfig.SERVO_CLAMP);
        testServo.setPosition(START_POS);
        telemetry.addData("servoPosition", START_POS);
        telemetry.addData("connection info", testServo.getConnectionInfo());
    }
    public void loop(){

        if (gamepad1.y) {
            position = START_POS;
            testServo.setPosition(position);
            telemetry.addData("servoPosition", position);
            telemetry.addData("connection info", testServo.getConnectionInfo());
        }
        if (gamepad1.a) {
            position = CLOSE_POS;
            testServo.setPosition(position);
            telemetry.addData("servoPosition", position);
        }

    }
}
