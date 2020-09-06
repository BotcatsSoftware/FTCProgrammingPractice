package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "gaming")
class gaming extends OpMode {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft= null;
    private DcMotor backRight = null;
    private Servo backServo = null;

    public void init(){
    frontLeft = hardwareMap.dcMotor.get("front_left_motor");
    frontRight = hardwareMap.dcMotor.get("front_right_motor");
    backLeft = hardwareMap.dcMotor.get("back_left_motor");
    backRight = hardwareMap.dcMotor.get("back_right_motor");
    backServo = hardwareMap.servo.get("back_servo");

    frontRight.setDirection(DcMotor.Direction.REVERSE);
    backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop(){
    while(gamepad1.y){
        backLeft.setPower(1);       //driving forward
        backRight.setPower(1);
        frontLeft.setPower(1);
        frontRight.setPower(1);
    }
    while(gamepad1.x){
        frontLeft.setPower(-1);
        frontRight.setPower(1);     //driving left horizontally
        backLeft.setPower(1);
        backRight.setPower(-1);
    }
    while(gamepad1.a){
        frontLeft.setPower(-1);
        frontRight.setPower(-1);   //moving backwards
        backLeft.setPower(-1);
        backRight.setPower(-1);
    }
    while(gamepad1.b){
        frontLeft.setPower(1);
        frontRight.setPower(-1);       //driving right horizontally
        backLeft.setPower(-1);
        backRight.setPower(1);
    }
    while(gamepad1.y && gamepad1.b){
        frontLeft.setPower(1);           //diagonal up and right
        backRight.setPower(1);
    }
    while(gamepad1.b && gamepad1.x){
        frontLeft.setPower(-1);          //diagonal motion down and left
        backRight.setPower(-1);
    }
     while(gamepad1.y && gamepad1.x){
         frontRight.setPower(1);           //diagonal up and left
         backLeft.setPower(1);
    }
     while(gamepad1.b && gamepad1.a){
         frontRight.setPower(-1);          //diagonal motion down and right
         backLeft.setPower(-1);
        }
     while(gamepad1.y && gamepad1.b && gamepad1.a){
         frontLeft.setPower(1);          //rotate about back right axis
         backLeft.setPower(1);
     }
     while(gamepad1.y && gamepad1.a && gamepad1.x){
         frontRight.setPower(1);            // rotate about back left axis
         backRight.setPower(1);
     }
     while(gamepad1.y && gamepad1.a && gamepad1.x && gamepad1.b){
         frontLeft.setPower(1);
         frontRight.setPower(-1);
         backRight.setPower(-1);
         backLeft.setPower(-1);
     }
    while(gamepad1.y && gamepad1.x && gamepad1.b){
        backServo.setPosition(180);
    }

     }





}
