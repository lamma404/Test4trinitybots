package org.firstinspires.ftc.teamcode.auton_drvctrl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.elevator.ElevatorConstants;

@TeleOp(group = "PowerPlay_Driver")
public class PP_DrvCtrl_OneDriver extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    /* drivetrain */
    private MecanumDrive m_drivetrain = null;
    private double m_maxPower = 0;  //speed - voltage
//    private double MAX_TIME = 120;       // time elapsed
    private double m_leftFrontPower, m_rightFrontPower, m_leftBackPower, m_rightBackPower = m_maxPower;

    /* clamp */
    private Servo m_clampServo = null;

    /* elevator */
    private DcMotorEx m_elevatorMotor = null;

    @Override
    public void init()
    {
        /* initialize drivetrain */
        m_drivetrain = new MecanumDrive(hardwareMap);
        m_drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_drivetrain.setMotorPowers(0,0,0,0);

        /* initialize clamp */
        m_clampServo = hardwareMap.get(Servo.class, BotConfig.SERVO_CLAMP);
        m_clampServo.setPosition(BotConfig.SERVO_OPEN_POS);

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

        runtime.reset();
    }

    @Override
    public void loop()
    {
        // drivetrain
        calDriveTrainPower();
        m_drivetrain.setMotorPowers(m_leftFrontPower, m_rightFrontPower, m_leftBackPower,m_rightBackPower);

        // clamp
        if (gamepad1.right_bumper) {
            m_clampServo.setPosition(BotConfig.SERVO_OPEN_POS);
            telemetry.addData("servoPosition", BotConfig.SERVO_OPEN_POS);
            telemetry.addData("connection info", m_clampServo.getConnectionInfo());
        }
        if (gamepad1.left_bumper) {
            m_clampServo.setPosition(BotConfig.SERVO_CLOSE_POS);
            telemetry.addData("servoPosition", BotConfig.SERVO_CLOSE_POS);
        }
        // elevator
        if (gamepad1.y) {   // medium junction
            moveElevator(BotConfig.MED_JUNCTION+1);
        }
        if (gamepad1.b) {  // low junction
            moveElevator(BotConfig.LOW_JUNCTION + 1);
        }
        if (gamepad1.a){  // ground
            moveElevator(0);
        }
        if (gamepad1.dpad_up){ // lift the elevator 1 inch by 1 inch
            int curr = m_elevatorMotor.getCurrentPosition();
            moveElevator(ElevatorConstants.encoderTicksToInches(curr)+1);
//            moveElevator(m_elevator_pos+1);
        }
        if (gamepad1.dpad_down) // drop the elevator 1 inch by 1 inch
        {
//            moveElevator(m_elevator_pos-1);
            int curr = m_elevatorMotor.getCurrentPosition();
            moveElevator(ElevatorConstants.encoderTicksToInches(curr)-1);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", m_leftFrontPower, m_rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", m_leftBackPower, m_rightBackPower);
        telemetry.update();
    }

    private void calDriveTrainPower()
    {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.right_stick_x;
        double yaw     =  gamepad1.left_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        m_leftFrontPower  = axial + lateral + yaw;
        m_rightFrontPower = axial + lateral - yaw;
        m_leftBackPower = axial - lateral + yaw;
        m_rightBackPower  = axial - lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        m_maxPower = Math.max(Math.abs(m_leftFrontPower), Math.abs(m_rightFrontPower));
        m_maxPower = Math.max(m_maxPower, Math.abs(m_leftBackPower));
        m_maxPower = Math.max(m_maxPower, Math.abs(m_rightBackPower));

        if (m_maxPower > 1.0) {
            m_leftFrontPower  /= m_maxPower;
            m_rightFrontPower /= m_maxPower;
            m_leftBackPower /= m_maxPower;
            m_rightBackPower  /= m_maxPower;
        }

    }
    private void moveElevator(double targetPos)
    {
        if ((targetPos < ElevatorConstants.MIN_HEIGHT) || (targetPos > ElevatorConstants.MAX_HEIGHT))
            return;

        m_elevatorMotor.setTargetPosition(ElevatorConstants.encoderInchesToTicks(targetPos));
        m_elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_elevatorMotor.setVelocity(ElevatorConstants.MAX_VELOCITY * 1.0 );
    }
    @Override
    public void stop()
    {
        m_drivetrain.setMotorPowers(0,0,0,0);
        m_clampServo.setPosition(BotConfig.SERVO_OPEN_POS);
        m_elevatorMotor.setVelocity(0);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
