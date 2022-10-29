package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.auton_drvctrl.BotConfig;

import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
public class MecanumDrive {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    public MecanumDrive(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, BotConfig.MOTOR_LEFT_FRONT);
        leftRear = hardwareMap.get(DcMotorEx.class, BotConfig.MOTOR_LEFT_BACK);
        rightRear = hardwareMap.get(DcMotorEx.class, BotConfig.MOTOR_RIGHT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, BotConfig.MOTOR_RIGHT_FRONT);

        leftFront.setDirection(BotConfig.MOTOR_LEFT_FRONT_DIR);
        leftRear.setDirection(BotConfig.MOTOR_LEFT_BACK_DIR);
        rightFront.setDirection(BotConfig.MOTOR_RIGHT_FRONT_DIR);
        rightRear.setDirection(BotConfig.MOTOR_RIGHT_BACK_DIR);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

    }
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }
}
