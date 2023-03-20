package org.firstinspires.ftc.teamcode.Autonomous.RoadRunner

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Autonomous.AprilTagDetectionPipeline
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.drive.DriveConstants


@Autonomous(name = "RRTest+camKT", group = "auto")
class Tester : LinearOpMode() {
    //not camera stuff
    var BLmotor: DcMotorEx? = null
    var BRmotor: DcMotorEx? = null
    var FLmotor: DcMotorEx? = null
    var FRmotor: DcMotorEx? = null
    var Xrail: DcMotorEx? = null
    var gripperL: Servo? = null
    var gripperR: Servo? = null
    var TicksPerRevolution = 537.7

    // diameter Millimeters
    var DiameterMM = 96.0

    // diameter Inches
    var DiameterIn = DiameterMM / 25.4 //3.7795275590551185 Inches

    // Circumference
    var Circumference = DiameterIn * Math.PI //11.873736013567724 Inches

    // Finding ticks per inch
    var TicksPerInch = TicksPerRevolution / Circumference //45.28482015985433 Inches

    //**guessed values**
    //Ticks per degree
    var TicksPerDegree = 7.52

    //strafe num
    var StrafeNum = 56.0

    //end of not camera stuff
    //camera stuff
    var camera: OpenCvCamera? = null
    var aprilTagDetectionPipeline: AprilTagDetectionPipeline? = null

    val FEET_PER_METER = 3.28084


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    var fx = 578.272
    var fy = 578.272
    var cx = 402.145
    var cy = 221.506

    // UNITS ARE METERS
    var tagsize = 0.166

    // Tag ID 1, 2, and 3 from the 36h11 family
    var LEFT = 1
    var MIDDLE = 2
    var RIGHT = 3
    var tagOfInterest: AprilTagDetection? = null

    //end of camera stuff
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        //getting motors
        BLmotor = hardwareMap.get(DcMotorEx::class.java, "BLmotor")
        FLmotor = hardwareMap.get(DcMotorEx::class.java, "FLmotor")
        BRmotor = hardwareMap.get(DcMotorEx::class.java, "BRmotor")
        FRmotor = hardwareMap.get(DcMotorEx::class.java, "FRmotor")
        Xrail = hardwareMap.get(DcMotorEx::class.java, "Xrail")

        //stops motors when not moving

        BLmotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BLmotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FLmotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BRmotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FRmotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        Xrail?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE


        //motor directions
        BLmotor?.direction = DcMotorSimple.Direction.FORWARD
        FLmotor?.direction = DcMotorSimple.Direction.FORWARD
        BRmotor?.direction = DcMotorSimple.Direction.REVERSE
        FRmotor?.direction = DcMotorSimple.Direction.REVERSE
        Xrail?.direction = DcMotorSimple.Direction.FORWARD

        //getting servos
        gripperL = hardwareMap.get(Servo::class.java, "gripperL")
        gripperR = hardwareMap.get(Servo::class.java, "gripperR")

        //servo directions
        gripperR?.direction = Servo.Direction.FORWARD
        gripperL?.direction = Servo.Direction.REVERSE

        //init


        //end of not camera stuff

        //back to camera stuff
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "Webcam 1"
            ), cameraMonitorViewId
        )
        aprilTagDetectionPipeline = AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy)
        (camera as OpenCvWebcam?)?.setPipeline(aprilTagDetectionPipeline)
        (camera as OpenCvWebcam?)?.openCameraDeviceAsync(object :
            OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                (camera as OpenCvWebcam?)?.startStreaming(
                    640,
                    480,
                    OpenCvCameraRotation.SIDEWAYS_LEFT
                )
            }

            override fun onError(errorCode: Int) {}
        })
        telemetry.msTransmissionInterval = 50

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */while (!isStarted && !isStopRequested) {
            val currentDetections = aprilTagDetectionPipeline!!.latestDetections
            if (currentDetections.size != 0) {
                var tagFound = false
                for (tag in currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag
                        tagFound = true
                        break
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:")
                    tagToTelemetry(tagOfInterest)
                } else {
                    telemetry.addLine("Don't see tag of interest :(")
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)")
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:")
                        tagToTelemetry(tagOfInterest)
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(")
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)")
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:")
                    tagToTelemetry(tagOfInterest)
                }
            }
            telemetry.update()
            sleep(20)
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n")
            tagToTelemetry(tagOfInterest)
            telemetry.update()
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(")
            telemetry.update()
        }



        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest!!.id == LEFT) {
            //trajectory

        }else if (tagOfInterest!!.id == MIDDLE) {
            //trajectory

        } else if (tagOfInterest!!.id == RIGHT) {
            //trajectory

        }
    }

    fun tagToTelemetry(detection: AprilTagDetection?) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection!!.id))
        telemetry.addLine(
            String.format(
                "Translation X: %.2f feet",
                detection.pose.x * FEET_PER_METER
            )
        )
        telemetry.addLine(
            String.format(
                "Translation Y: %.2f feet",
                detection.pose.y * FEET_PER_METER
            )
        )
        telemetry.addLine(
            String.format(
                "Translation Z: %.2f feet",
                detection.pose.z * FEET_PER_METER
            )
        )
        telemetry.addLine(
            String.format(
                "Rotation Yaw: %.2f degrees", Math.toDegrees(
                    detection.pose.yaw
                )
            )
        )
        telemetry.addLine(
            String.format(
                "Rotation Pitch: %.2f degrees", Math.toDegrees(
                    detection.pose.pitch
                )
            )
        )
        telemetry.addLine(
            String.format(
                "Rotation Roll: %.2f degrees", Math.toDegrees(
                    detection.pose.roll
                )
            )
        )
    }
}