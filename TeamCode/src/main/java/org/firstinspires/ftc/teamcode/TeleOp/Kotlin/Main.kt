package org.firstinspires.ftc.teamcode.TeleOp.Kotlin

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

/*
Tester, this is where we write and test all of our code,
ALWAYS test and write new code here, no where else
 */

@TeleOp(name = "MainKT", group = "Controlled")
class Main : OpMode() {
    var runtime = ElapsedTime()

    //pre init
    //motors
    var FLmotor: DcMotor? = null
    var FRmotor: DcMotor? = null
    var BLmotor: DcMotor? = null
    var BRmotor: DcMotor? = null
    var XrailL: DcMotor? = null
    var XrailR: DcMotor? = null
    var longGripper: DcMotor? = null

    //Servos
    var gripperL: Servo? = null
    var gripperR: Servo? = null

    //speed variables
    var TurnSpeed = .7
    var drivespeed = .7

    //servo positions
    override fun init() {

        //getting motors
        BLmotor = hardwareMap.get(DcMotor::class.java, "BLmotor")
        FLmotor = hardwareMap.get(DcMotor::class.java, "FLmotor")
        BRmotor = hardwareMap.get(DcMotor::class.java, "BRmotor")
        FRmotor = hardwareMap.get(DcMotor::class.java, "FRmotor")
        XrailL = hardwareMap.get(DcMotor::class.java, "XrailL")
        XrailR = hardwareMap.get(DcMotor::class.java, "XrailR")
        longGripper = hardwareMap.get(DcMotor::class.java, "longGripper")

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

        //servo directions
        gripperR?.direction = Servo.Direction.FORWARD
        gripperL?.direction = Servo.Direction.FORWARD

        //init
        gripperL?.position = (.9)
        gripperR?.position = (.5)
        runtime.reset()

    }

    override fun loop() {
        if (gamepad2.a) {
            gripperL?.position = .1
            gripperL?.position = .1
        } else {
            gripperL?.position = .9
            gripperL?.position = .5
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
        FRmotor?.power = ((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed - (gamepad1.left_stick_y + gamepad1.right_stick_x) * drivespeed)    }
}



