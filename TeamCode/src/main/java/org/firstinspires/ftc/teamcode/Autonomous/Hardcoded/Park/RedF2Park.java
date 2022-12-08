/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous.Hardcoded.Park;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name="RedLeftPark",group="auto")
public class RedF2Park extends LinearOpMode {
    //not camera stuff
    DcMotorEx BLmotor, BRmotor, FLmotor, FRmotor, Xrail;
    Servo gripperL, gripperR;
    double TicksPerRevolution = 537.7;
    // diameter Millimeters
    double DiameterMM = 96;
    // diameter Inches
    double DiameterIn = DiameterMM / 25.4; //3.7795275590551185 Inches
    // Circumference
    double Circumference = DiameterIn * PI; //11.873736013567724 Inches
    // Finding ticks per inch
    double TicksPerInch = TicksPerRevolution / Circumference; //45.28482015985433 Inches

    //**guessed values**
    //Ticks per degree
    double TicksPerDegree = 7.52;
    //strafe num
    double StrafeNum = 56;
    //end of not camera stuff

    //camera stuff
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, and 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;
    //end of camera stuff

    @Override
    public void runOpMode() throws InterruptedException {
        //getting motors
        BLmotor = hardwareMap.get(DcMotorEx.class, "BLmotor");
        FLmotor = hardwareMap.get(DcMotorEx.class, "FLmotor");
        BRmotor = hardwareMap.get(DcMotorEx.class, "BRmotor");
        FRmotor = hardwareMap.get(DcMotorEx.class, "FRmotor");
        Xrail = hardwareMap.get(DcMotorEx.class, "Xrail");

        //stops motors when not moving
        BLmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Xrail.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //motor directions
        BLmotor.setDirection(DcMotorEx.Direction.FORWARD);
        FLmotor.setDirection(DcMotorEx.Direction.FORWARD);
        BRmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FRmotor.setDirection(DcMotorEx.Direction.REVERSE);
        Xrail.setDirection(DcMotorEx.Direction.FORWARD);

        //getting servos
        gripperL = hardwareMap.get(Servo.class, "gripperL");
        gripperR = hardwareMap.get(Servo.class, "gripperR");

        //servo directions
        gripperR.setDirection(Servo.Direction.FORWARD);
        gripperL.setDirection(Servo.Direction.REVERSE);

        //init


        //end of not camera stuff

        //back to camera stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null){
            //trajectory
            forward(.3,4);
            Sright(.3,25);
            backwards(.3,3);
        } else if(tagOfInterest.id == LEFT)
        {
            //trajectory
            forward(.4,27);
            Sright(.3,20);

        } else if (tagOfInterest.id == MIDDLE) {
            //trajectory
            forward(.4,30);

        } else if (tagOfInterest.id == RIGHT) {
            //trajectory
            forward(.4,30);
            Sleft(.3,20);

        }

    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    //end of camera stuff
    //not camera stuff
    public void forward(double power, double Inches) {
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
    public void backwards(double power, double Inches) {
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
    public void Sleft(double power, double Inches) {
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
    public void Sright(double power, double Inches) {
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
    public void DFright(double power, double Inches) {
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
    public void DFleft(double power, double Inches) {
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
    public void DBright(double power, double Inches) {
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
    public void DBleft(double power, double Inches) {
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
    public void TurnRight(double power, double degrees) {

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

        while (opModeIsActive() && (FRmotor.isBusy() &&
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

    public void TurnLeft(double power, double degrees) {

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

        while (opModeIsActive() && (FRmotor.isBusy() &&
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

        Xrail.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Xrail.setTargetPosition(distance);
        Xrail.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Xrail.setPower(power);

        while (Xrail.isBusy()) {
        }
        Xrail.setPower(0);
        Xrail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void grippers(double position){
        gripperL.setPosition(position);
        gripperR.setPosition(position);
    }


}

