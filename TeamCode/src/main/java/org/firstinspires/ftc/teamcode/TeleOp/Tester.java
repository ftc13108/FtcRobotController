package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Tester, this is where we write and test all of our code,
ALWAYS test and write new code here, no where else
 */

@TeleOp(name="Tester", group="Controlled")
public class Tester extends LinearOpMode {
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
    double TurnSpeed = .5;
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
        openGrippers();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(gamepad2.a){
                closeGrippers();
            }
            else{
                openGrippers();
            }

            if(gamepad2.b){
                XrailMove(.3,5);
            }

            if(gamepad2.y){
                XrailMove(.3,5);
            }
            if(gamepad2.x){
                XrailMove(.3,5);
            }

            //joystick Xrail
            Xrail.setPower(gamepad2.left_stick_y * .8);

            //drive
            BLmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) + (gamepad1.left_stick_y + (-gamepad1.right_stick_x))*(drivespeed));
            FLmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) + (gamepad1.left_stick_y - (-gamepad1.right_stick_x))*(drivespeed));
            BRmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) - (gamepad1.left_stick_y - (-gamepad1.right_stick_x))*(drivespeed));
            FRmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) - (gamepad1.left_stick_y + (-gamepad1.right_stick_x))*(drivespeed));
            //end of drive
        }

    }
    public void XrailMove(double power, double Inches) {
        double TicksPerRevolution = 537.7;
        // diameter Millimeters
        double DiameterMM = 38.1;
        // diameter Inches
        double DiameterIn = DiameterMM / 25.4; //3.7795275590551185 Inches
        // Circumference
        double Circumference = DiameterIn * PI; //11.873736013567724 Inches
        // Finding ticks per inch
        double TicksPerInch = TicksPerRevolution / Circumference; //45.28482015985433 Inches

        int distance = (int) (Inches * TicksPerInch);

        Xrail.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); //resetting the encoder to clear it
        Xrail.setTargetPosition(distance); //giving the encoder a target position
        Xrail.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); //telling it to go to that position
        Xrail.setPower(power); //giving it the power

        while (Xrail.isBusy()) {
        }
        Xrail.setPower(0); //just for when its done
        Xrail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void closeGrippers(){
        gripperL.setPosition(.1);
        gripperR.setPosition(.1);
    }
    public void openGrippers(){
        gripperL.setPosition(.9);
        gripperR.setPosition(.9);
    }
}




