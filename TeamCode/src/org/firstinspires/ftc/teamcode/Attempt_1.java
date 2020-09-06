package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "attempt_1", group = "attempts")
public class Attempt_1 extends LinearOpMode {
    public void runOpMode()

    {
        DcMotor left = null;
        DcMotor right = null;
        DcMotor back_left = null;
        DcMotor back_right = null;
        Servo back_servo = null;
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setPower(1);
        right.setPower(-1);

        left.setPower(-1);
        right.setPower(1);

        left.setPower(1);
        back_left.setPower(-1);
        right.setPower(-1);
        back_right.setPower(1);

        left.setPower(-1);
        back_left.setPower(1);
        right.setPower(1);
        back_right.setPower(-1);

        back_servo.setPosition(180);

    }
}





