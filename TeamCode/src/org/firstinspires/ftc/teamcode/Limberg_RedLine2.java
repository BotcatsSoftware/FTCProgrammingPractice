package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "RedLine_1", group = "RedLine")
public class Limberg_RedLine2 extends LinearOpMode {
    public void runOpMode() {
        DcMotor left = null;
        DcMotor right = null;
        DcMotor back_left = null;
        DcMotor back_right = null;
        Servo back_servo = null;
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color sensor");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int enc = left.getCurrentPosition();
        while (left.getCurrentPosition() < enc + 5000) {
            telemetry.addData( "Encoder: ", left.getCurrentPosition() );

        }

    }
}
