package org.firstinspires.ftc.teamcode.auton_drvctrl;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
/*
 *  1. 4 x motors (312 rpm) on drivetrain ==> control hub / motors
 *       port 0 - left_back_drive
 *       port 1 - right_back_drive
 *       port 2 - left_front_drive
 *       port 3 - right_front_drive
* */
@Disabled
@TeleOp(group = "PowerPlay_Driver")
public class PP_DrvCtrl_Sim extends OpMode {

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

        runtime.reset();
    }

    @Override
    public void loop()
    {
        // drivetrain
        calDriveTrainPower();
        m_drivetrain.setMotorPowers(m_leftFrontPower, m_rightFrontPower, m_leftBackPower,m_rightBackPower);

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
    @Override
    public void stop()
    {
        m_drivetrain.setMotorPowers(0,0,0,0);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
