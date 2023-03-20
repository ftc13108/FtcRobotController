package org.firstinspires.ftc.teamcode.TeleOp.Java;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware;


/*
Tester, this is where we write and test all of our code,
ALWAYS test and write new code here, no where else
 */

@TeleOp(name="Tester", group="Controlled")
public class Tester extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    //pre init
    //motors
    public static DcMotorEx FLmotor, FRmotor, BLmotor, BRmotor, XrailL, XrailR, longGripper;

    //Servos
    public static Servo gripperL, gripperR, left, middle, right;

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
        XrailL = hardwareMap.get(DcMotorEx.class, "XrailL");
        XrailR = hardwareMap.get(DcMotorEx.class, "XrailR");
        longGripper = hardwareMap.get(DcMotorEx.class, "longGripper");

        //making them brake
        BLmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        XrailL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        XrailR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        longGripper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //motor directions
        BLmotor.setDirection(DcMotorEx.Direction.FORWARD);
        FLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        BRmotor.setDirection(DcMotorEx.Direction.FORWARD);
        FRmotor.setDirection(DcMotorEx.Direction.REVERSE);
        XrailL.setDirection(DcMotorEx.Direction.FORWARD);
        XrailR.setDirection(DcMotorEx.Direction.FORWARD);
        longGripper.setDirection(DcMotorEx.Direction.FORWARD);

        //getting servos
        gripperL = hardwareMap.get(Servo.class, "gripperL");
        gripperR = hardwareMap.get(Servo.class, "gripperR");
        left = hardwareMap.get(Servo.class, "left");
        middle = hardwareMap.get(Servo.class, "middle");
        right = hardwareMap.get(Servo.class, "right");



        //servo directions
        gripperR.setDirection(Servo.Direction.FORWARD);
        gripperL.setDirection(Servo.Direction.REVERSE);
        left.setDirection(Servo.Direction.FORWARD);
        middle.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        //init
        gripperL.setPosition(.9);
        gripperR.setPosition(.5);

        double MAX_POSITION = 0.5;

        PodsUp();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            PodsUp();

            if(gamepad2.right_bumper){
                gripperL.setPosition(.1);
                gripperR.setPosition(.1);
            }
            else{
                gripperL.setPosition(.9);
                gripperR.setPosition(.5);
            }

            XrailL.setPower(gamepad2.left_stick_y * 1);
            XrailR.setPower(gamepad2.left_stick_y * 1);

            longGripper.setPower(gamepad2.right_stick_y * 1);

            //drive
            BLmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * (TurnSpeed)) + (gamepad1.left_stick_y + gamepad1.right_stick_x)*(drivespeed));
            FLmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * (TurnSpeed)) + (gamepad1.left_stick_y - gamepad1.right_stick_x)*(drivespeed));
            BRmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * (TurnSpeed)) - (gamepad1.left_stick_y - gamepad1.right_stick_x)*(drivespeed));
            FRmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * (TurnSpeed)) - (gamepad1.left_stick_y + gamepad1.right_stick_x)*(drivespeed));
            //end of drive
        }
    }
    public static void grip() {
        gripperL.setPosition(.1);
        gripperR.setPosition(.1);
    }

    public static void noGrip() {
        gripperL.setPosition(.9);
        gripperR.setPosition(.5);
    }

    public static void PodsUp() {
        left.setPosition(.9);
        middle.setPosition(.05);
        right.setPosition(.75);
    }

    public static void PodsDown() {
        left.setPosition(.725);
        middle.setPosition(.65);
        right.setPosition(.95);
    }
}

