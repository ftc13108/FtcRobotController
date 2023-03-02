package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Main", group="Controlled")
public class Main extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    //pre init
    //motors
    DcMotorEx FLmotor;
    DcMotorEx FRmotor;
    DcMotorEx BLmotor;
    DcMotorEx BRmotor;
    DcMotorEx Xrail;

    //Servos
    Servo gripperL;
    Servo gripperR;

    //speed variables
    double TurnSpeed = .7;
    double drivespeed = .7;

    //servo positions

    public void runOpMode() {

        //getting motors
        BLmotor = hardwareMap.get(DcMotorEx.class, "BLmotor");
        FLmotor = hardwareMap.get(DcMotorEx.class, "FLmotor");
        BRmotor = hardwareMap.get(DcMotorEx.class, "BRmotor");
        FRmotor = hardwareMap.get(DcMotorEx.class, "FRmotor");
        Xrail = hardwareMap.get(DcMotorEx.class, "Xrail");



        BRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Xrail.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //motor directions
        BLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        BRmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FRmotor.setDirection(DcMotorEx.Direction.REVERSE);
        Xrail.setDirection(DcMotorEx.Direction.REVERSE);

        //getting servos
        gripperL = hardwareMap.get(Servo.class,"gripperL");
        gripperR = hardwareMap.get(Servo.class,"gripperR");

        //servo directions
        gripperR.setDirection(Servo.Direction.FORWARD);
        gripperL.setDirection(Servo.Direction.REVERSE);

        //init
        gripperL.setPosition(.9);
        gripperR.setPosition(.5);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(gamepad2.a){
                gripperL.setPosition(.1);
                gripperR.setPosition(.1);
            }
            else{
                gripperL.setPosition(.9);
                gripperR.setPosition(.5);
            }


            Xrail.setPower(gamepad2.left_stick_y * 1);

            //drive
            BLmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * (TurnSpeed)) + (gamepad1.left_stick_y + gamepad1.right_stick_x)*(drivespeed));
            FLmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * (TurnSpeed)) + (gamepad1.left_stick_y - gamepad1.right_stick_x)*(drivespeed));
            BRmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * (TurnSpeed)) - (gamepad1.left_stick_y - gamepad1.right_stick_x)*(drivespeed));
            FRmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * (TurnSpeed)) - (gamepad1.left_stick_y + gamepad1.right_stick_x)*(drivespeed));
            //end of drive
        }
    }
}

