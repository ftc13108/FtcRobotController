package org.firstinspires.ftc.teamcode.TeleOp.Kotlin

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

/*
Tester, this is where we write and test all of our code,
ALWAYS test and write new code here, no where else
 */

@TeleOp(name = "TesterKt", group = "Controlled")
class Tester : LinearOpMode() {
    var runtime = ElapsedTime()

    //pre init
    //motors
    var BLmotor: DcMotorEx? = null
    var FLmotor: DcMotorEx? = null
    var BRmotor: DcMotorEx? = null
    var FRmotor: DcMotorEx? = null
    var XrailL: DcMotorEx? = null
    var XrailR: DcMotorEx? = null
    var longGripper: DcMotorEx? = null

    //Servos
    var gripperL: Servo? = null
    var gripperR: Servo? = null
    var left: Servo? = null
    var middle: Servo? = null
    var right: Servo? = null


    //speed variables
    var TurnSpeed = .7
    var drivespeed = .7

    override fun runOpMode() {
        //getting motors
        BLmotor = hardwareMap.get(DcMotorEx::class.java, "BLmotor")
        FLmotor = hardwareMap.get(DcMotorEx::class.java, "FLmotor")
        BRmotor = hardwareMap.get(DcMotorEx::class.java, "BRmotor")
        FRmotor = hardwareMap.get(DcMotorEx::class.java, "FRmotor")
        XrailL = hardwareMap.get(DcMotorEx::class.java, "XrailL")
        XrailR = hardwareMap.get(DcMotorEx::class.java, "XrailR")
        longGripper = hardwareMap.get(DcMotorEx::class.java, "longGripper")

        //making them brake
        BLmotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FLmotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BRmotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FRmotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        XrailL?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        XrailR?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        longGripper?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        //motor directions
        BLmotor?.direction = DcMotorSimple.Direction.REVERSE
        FLmotor?.direction = DcMotorSimple.Direction.REVERSE
        BRmotor?.direction = DcMotorSimple.Direction.REVERSE
        FRmotor?.direction = DcMotorSimple.Direction.REVERSE
        XrailL?.direction = DcMotorSimple.Direction.FORWARD
        XrailR?.direction = DcMotorSimple.Direction.FORWARD
        longGripper?.direction = DcMotorSimple.Direction.FORWARD

        //getting servos
        gripperL = hardwareMap.get(Servo::class.java, "gripperL")
        gripperR = hardwareMap.get(Servo::class.java, "gripperR")
        left = hardwareMap.get(Servo::class.java, "left")
        middle = hardwareMap.get(Servo::class.java, "middle")
        right = hardwareMap.get(Servo::class.java, "right")


        //servo directions

        //servo directions
        gripperR?.direction = Servo.Direction.FORWARD
        gripperL?.direction = Servo.Direction.REVERSE
        left?.direction = Servo.Direction.FORWARD
        middle?.direction = Servo.Direction.FORWARD
        right?.direction = Servo.Direction.FORWARD



        //init

        //servo positions
        gripperL?.position = (.9)
        gripperR?.position = (.5)
        left?.position = (.5)
        middle?.position = (.5)
        right?.position = (.5)


        runtime.reset()

        if (gamepad2.a) {
            gripperL?.position = .1
            gripperL?.position = .1
            telemetry.addLine("gripping")
            telemetry.update()
        } else {
            gripperL?.position = .9
            gripperL?.position = .5
            telemetry.addLine("not gripping :( ")
            telemetry.update()
        }

        //move the slides
        XrailL?.power = ((gamepad2.left_stick_y * 1).toDouble())
        XrailR?.power = ((gamepad2.left_stick_y * 1).toDouble())

        //move the big rail thingy
        longGripper?.power = ((gamepad2.right_stick_y * 1).toDouble())

        //drive
        BLmotor?.power = ((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed + (gamepad1.left_stick_y + gamepad1.right_stick_x) * drivespeed)
        FLmotor?.power = ((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed + (gamepad1.left_stick_y - gamepad1.right_stick_x) * drivespeed)
        BRmotor?.power = ((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed - (gamepad1.left_stick_y - gamepad1.right_stick_x) * drivespeed)
        FRmotor?.power = ((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed - (gamepad1.left_stick_y + gamepad1.right_stick_x) * drivespeed)
    }
}



