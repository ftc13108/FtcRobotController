package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
This is where I keep all of my methods and "dirty" code
basically just the unappealing to look at code and
simplifies writing my op modes
 */

@Config
public class Hardware extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    static double TicksPerRevolution = 537.7;
    // diameter Millimeters
    static double DiameterMM = 96;
    // diameter Inches
    static double DiameterIn = DiameterMM / 25.4; //3.7795275590551185 Inches
    // Circumference
    static double Circumference = DiameterIn * PI; //11.873736013567724 Inches
    // Finding ticks per inch
    static double TicksPerInch = TicksPerRevolution / Circumference; //45.28482015985433 Inches

    static double StrafeNum = 56;

    static double TicksPerDegree = 7.52;

    //motors
    public static DcMotorEx FLmotor, FRmotor, BLmotor, BRmotor, XrailL, XrailR, longGripper;

    //Servos
    public static Servo gripperL, gripperR, left, middle, right;

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
        BLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        BRmotor.setDirection(DcMotorEx.Direction.REVERSE);
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
        right.setDirection(Servo.Direction.REVERSE);



    }

    //servo methods
    public static void grip(){
        gripperL.setPosition(.1);
        gripperR.setPosition(.1);
    }
    public static void noGrip(){
        gripperL.setPosition(.9);
        gripperR.setPosition(.5);
    }
    public static void PodsUp(){
        left.setPosition(.3);
        middle.setPosition(.5);
        right.setPosition(.3);
    }
    public static void PodsDown(){
        left.setPosition(.3);
        middle.setPosition(.5);
        right.setPosition(.3);
    }


    //moves slides with encoders, not that gross
    public static void SlideMove(double power, double Inches) {
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

        XrailL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        XrailR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        XrailL.setTargetPosition(distance);
        XrailR.setTargetPosition(distance);

        XrailL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        XrailR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        XrailL.setPower(power);
        XrailR.setPower(power);

        while (XrailL.isBusy() && XrailR.isBusy()) {
        }
        XrailL.setPower(0);
        XrailR.setPower(0);

        XrailL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        XrailR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }






    /*ENCODER DRIVE METHODS
    The **real** dirty code
     */
    public static void forward(double power, double Inches) {
        int distance = (int) (Inches * TicksPerInch);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRmotor.setTargetPosition(distance);
        FLmotor.setTargetPosition(distance);
        BRmotor.setTargetPosition(distance);
        BLmotor.setTargetPosition(distance);

        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        FLmotor.setPower(power);
        FRmotor.setPower(power);
        BLmotor.setPower(power);
        BRmotor.setPower(power);

        while (FRmotor.isBusy() &&
                FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy()) {
        }

        FRmotor.setPower(0);
        FLmotor.setPower(0);
        BRmotor.setPower(0);
        BLmotor.setPower(0);
        FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    //backwards
    public static void backwards(double power, double Inches) {
        int distance = (int) (Inches * TicksPerInch);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRmotor.setTargetPosition(-distance);
        FLmotor.setTargetPosition(-distance);
        BRmotor.setTargetPosition(-distance);
        BLmotor.setTargetPosition(-distance);

        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        FRmotor.setPower(-power);
        FLmotor.setPower(-power);
        BRmotor.setPower(-power);
        BLmotor.setPower(-power);

        while (FRmotor.isBusy() &&
                FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy()) {
        }

        FRmotor.setPower(0);
        FLmotor.setPower(0);
        BRmotor.setPower(0);
        BLmotor.setPower(0);
        FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    //strafing stuff
    //strafe left
    public static void Sleft(double power, double Inches) {
        int distance = (int) (Inches * StrafeNum);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRmotor.setTargetPosition(distance);
        FLmotor.setTargetPosition(-distance);
        BRmotor.setTargetPosition(-distance);
        BLmotor.setTargetPosition(distance);

        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        FRmotor.setPower(power);
        FLmotor.setPower(-power);
        BRmotor.setPower(-power);
        BLmotor.setPower(power);

        while (FRmotor.isBusy() &&
                FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy()) {
        }

        FRmotor.setPower(0);
        FLmotor.setPower(0);
        BRmotor.setPower(0);
        BLmotor.setPower(0);
        FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    //strafe right
    public static void Sright(double power, double Inches) {
        int distance = (int) (Inches * StrafeNum);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRmotor.setTargetPosition(-distance);
        FLmotor.setTargetPosition(distance);
        BRmotor.setTargetPosition(distance);
        BLmotor.setTargetPosition(-distance);

        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        FRmotor.setPower(-power);
        FLmotor.setPower(power);
        BRmotor.setPower(power);
        BLmotor.setPower(-power);

        while (FRmotor.isBusy() &&
                FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy()) {
        }

        FRmotor.setPower(0);
        FLmotor.setPower(0);
        BRmotor.setPower(0);
        BLmotor.setPower(0);
        FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    //turning stuff
    //diagonal front right
    public static void DFright(double power, double Inches) {
        int distance = (int) (Inches * TicksPerInch);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        FRmotor.setTargetPosition(distance);
        BLmotor.setTargetPosition(distance);


        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        FRmotor.setPower(power);
        BLmotor.setPower(power);

        while (FRmotor.isBusy() && BLmotor.isBusy()) {
        }

        FRmotor.setPower(0);
        BLmotor.setPower(0);
        FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    //diagonal front left
    public static void DFleft(double power, double Inches) {
        int distance = (int) (Inches * TicksPerInch);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        FRmotor.setTargetPosition(distance);
        BLmotor.setTargetPosition(distance);


        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        FLmotor.setPower(power);
        BRmotor.setPower(power);

        while (FRmotor.isBusy() && BLmotor.isBusy()) {
        }

        FLmotor.setPower(0);
        BRmotor.setPower(0);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    //diagonal back right
    public static void DBright(double power, double Inches) {
        int distance = (int) (Inches * TicksPerInch);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        FRmotor.setTargetPosition(-distance);
        BLmotor.setTargetPosition(-distance);


        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        FRmotor.setPower(-power);
        BLmotor.setPower(-power);

        while (FRmotor.isBusy() && BLmotor.isBusy()) {
        }

        FRmotor.setPower(0);
        BLmotor.setPower(0);
        FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    //diagonal back left
    public static void DBleft(double power, double Inches) {
        int distance = (int) (Inches * TicksPerInch);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        FRmotor.setTargetPosition(-distance);
        BLmotor.setTargetPosition(-distance);


        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        FLmotor.setPower(-power);
        BRmotor.setPower(-power);

        while (FRmotor.isBusy() && BLmotor.isBusy()) {
        }

        FLmotor.setPower(0);
        BRmotor.setPower(0);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    //turning
    public static void TurnRight(double power, double degrees) {

        int distance = (int) (degrees * TicksPerDegree);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRmotor.setTargetPosition(-distance);
        FLmotor.setTargetPosition(distance);
        BRmotor.setTargetPosition(-distance);
        BLmotor.setTargetPosition(distance);

        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        FRmotor.setPower(-power);
        FLmotor.setPower(power);
        BRmotor.setPower(-power);
        BLmotor.setPower(power);

        while ((FRmotor.isBusy() &&
                FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy())) {
        }

        FRmotor.setPower(0);
        FLmotor.setPower(0);
        BRmotor.setPower(0);
        BLmotor.setPower(0);
        FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public static void TurnLeft(double power, double degrees) {

        int distance = (int) (degrees * TicksPerDegree);

        FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRmotor.setTargetPosition(distance);
        FLmotor.setTargetPosition(-distance);
        BRmotor.setTargetPosition(distance);
        BLmotor.setTargetPosition(-distance);

        FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        FRmotor.setPower(power);
        FLmotor.setPower(-power);
        BRmotor.setPower(power);
        BLmotor.setPower(-power);

        while ((FRmotor.isBusy() &&
                FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy())) {
        }

        FRmotor.setPower(0);
        FLmotor.setPower(0);
        BRmotor.setPower(0);
        BLmotor.setPower(0);
        FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


}

