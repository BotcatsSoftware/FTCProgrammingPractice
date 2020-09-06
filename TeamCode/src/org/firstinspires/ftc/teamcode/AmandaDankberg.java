package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@Autonomous(name = "Amandas cool code")
public class AmandaDankberg extends LinearOpMode {
    public void runOpMode(){
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        Servo servo = hardwareMap.servo.get("back_servo");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");

        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();
        while(opModeIsActive()){


            if(backDistance.getDistance(DistanceUnit.CM) < 5 && rightDistance.getDistance(DistanceUnit.CM) > 5){
                fl.setPower(-1);
                fr.setPower(1);
                bl.setPower(1);
                br.setPower(-1);
            }
            else if(rightDistance.getDistance(DistanceUnit.CM) < 5 && frontDistance.getDistance(DistanceUnit.CM) > 5){
                fl.setPower(-1);
                fr.setPower(-1);
                bl.setPower(-1);
                br.setPower(-1);
            }
            else if(frontDistance.getDistance(DistanceUnit.CM) < 5 && leftDistance.getDistance(DistanceUnit.CM) > 5){
                fl.setPower(1);
                fr.setPower(-1);
                bl.setPower(-1);
                br.setPower(1);
            }
            else if(leftDistance.getDistance(DistanceUnit.CM) < 5 && rightDistance.getDistance(DistanceUnit.CM) > 5){
                fl.setPower(1);
                fr.setPower(1);
                bl.setPower(1);
                br.setPower(1);
            }
            else{
                fl.setPower(1);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(1);
            }

            /*
            //Once the color sensor value reaches >165, the robot will stop at that point and start spinning
            if(colorSensor.red() >= 165){
                fl.setPower(1);
                fr.setPower(-1);
                bl.setPower(1);
                br.setPower(-1);
            }
            //moves forward if condition isn't met
            else{
                fl.setPower(-1);
                fr.setPower(-1);
                bl.setPower(-1);
                br.setPower(-1);
            }
            */
            telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Color Sensor: ", colorSensor.red());
            telemetry.update();

        }
    }
}
/*
//forward
servo.setPosition(0);
fl.setPower(-1);
fr.setPower(-1);
bl.setPower(-1);
br.setPower(-1);

//backwards
fl.setPower(1);
fr.setPower(1);
bl.setPower(1);
br.setPower(1);

//moves left
fl.setPower(1);
fr.setPower(-1);
bl.setPower(-1);
br.setPower(1);

//moves right
fl.setPower(-1);
fr.setPower(1);
bl.setPower(1);
br.setPower(-1);

//diagonal
fl.setPower(1);
fr.setPower(0);
bl.setPower(0);
br.setPower(1);
 */
