package org.firstinspires.ftc.teamcode.auton_drvctrl;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
* * [Hardware configuration]
 *  1. mecanum wheel drivetrain, 4 x 312rpm motors are NOT connected with encoder
 *  2. 3 dead wheels with encoders
 *  3. elevator structure is mounted in the middle of the bot
 *  4. 1 servo is built with gear box to control the clamp (directly interact with cone)
 *  5. 1 web camera is mounted at the back of the bot
 *
 *  Details
 *  1. 4 x motors (312 rpm) on drivetrain ==> control hub / motors
 *       port 0 - left_back_drive
 *       port 1 - right_back_drive
 *       port 2 - left_front_drive
 *       port 3 - right_front_drive
 *
 *  2. 3 x dead wheels  ==> control hub / motors
 *       left wheel:  port 0 - left_back_drive
 *       right wheel: port 1 - right_back_drive
 *       back wheel:  port 2 - left_front_drive
 *
 *  3. 1 motor (435 rpm) elevator structure ==> expansion hub
 *       port 0 - motor_elevator
 *
 *  4. servo for clamp ==> control hub
 *       port 0 - servo_clamp
 *
 *  5. web camera ==> control hub
 *      USB 2.0 - webcam_signal
* */
public class BotConfig {
    /* drivetrain */
    public static final String MOTOR_LEFT_FRONT  = "left_front_drive"  ; // expansion hub - port 0
    public static final String MOTOR_LEFT_BACK   = "left_back_drive"    ; // expansion hub - port 1

    public static final String MOTOR_RIGHT_FRONT = "right_front_drive";  // control hub  - port 0
    public static final String MOTOR_RIGHT_BACK  = "right_back_drive"  ; // control hub  - port 1

    public static final DcMotorSimple.Direction MOTOR_LEFT_FRONT_DIR = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction MOTOR_LEFT_BACK_DIR = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction MOTOR_RIGHT_FRONT_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction MOTOR_RIGHT_BACK_DIR = DcMotorSimple.Direction.FORWARD;

    /* clamp */
    public static final String SERVO_CLAMP = "servo_clamp";     // servo - control hub - port 0
    public static final double SERVO_CLOSE_POS = 1.0;           // clamp - CLOSE position
    public static final double SERVO_OPEN_POS  = 0.6;           // clamp - OPEN position

    /* camera */
    public static final String WEBCAM_SIGNAL = "webcam_signal"; // control hub - USB 2.0

    /* elevator */
    public static final String MOTOR_ELEVATOR = "motor_elevator"    ; // control hub - port 2

    /* game related */
    public static double MED_JUNCTION = 23;
    public static double LOW_JUNCTION = 13;
}
