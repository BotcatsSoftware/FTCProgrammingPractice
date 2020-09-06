package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


// this is the bare bones LinearOpMode code if anyone wanted it

//you could put @TeleOp here instead of @Autonomous and it would still work, @Autonomous does not have to go with OpMode

@TeleOp(name = "Example Linear OpMode")
public class Rohan_S_2 extends LinearOpMode {
    DcMotor left = hardwareMap.dcMotor.get("Left_Motor");
    DcMotor right = hardwareMap.dcMotor.get("Right_Motor");
    Servo servo1 = hardwareMap.servo.get("servo1");

    CRServo servo2 = hardwareMap.crservo.get("servo2");



    public void runOpMode(){
        left.setDirection(DcMotor.Direction.FORWARD);

        left.setPower(1);

        left.setPower(-1);

        left.setPower(0.5);

        servo2.setPower(1);


    }


}
