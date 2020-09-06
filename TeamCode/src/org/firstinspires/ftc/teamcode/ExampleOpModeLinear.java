package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


// this is the bare bones LinearOpMode code if anyone wanted it

//you could put @TeleOp here instead of @Autonomous and it would still work, @Autonomous does not have to go with OpMode

@TeleOp(name = "Example Linear OpMode")
public class ExampleOpModeLinear extends LinearOpMode {
    DcMotor topLeft = hardwareMap.dcMotor.get("top_Left");
    DcMotor topRight = hardwareMap.dcMotor.get("top_Right");
    DcMotor bottomRight = hardwareMap.dcMotor.get("bottom_Right");
    DcMotor bottomLeft = hardwareMap.dcMotor.get("bottom_Left");
    Servo servo1 = hardwareMap.servo.get("servo1");

    CRServo crServo = hardwareMap.crservo.get("crServo");


    public void runOpMode(){
        topLeft.setDirection(DcMotor.Direction.FORWARD);

        topLeft.setPower(1);
        topRight.setPower(1);
        bottomRight.setPower(1);
        bottomLeft.setPower(1);
        servo1.setPosition(180);

        crServo.setPower(1);




    }


}
