package org.firstinspires.ftc.team16910.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Spaghetti")
public class Spaghetti extends OpMode {

    SpaghettiHardware robot;

    public void init() {
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);
    }

    public void start() {

    }

    public void loop() {

        // Carousel Code
        if (gamepad2.left_bumper) robot.spinnyWheel.setPower(-0.6);
        else if (gamepad2.right_bumper) robot.spinnyWheel.setPower(0.6);
        else robot.spinnyWheel.setPower(0.0);


        // Mecanum drivecode
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1 ) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        robot.leftFront.setPower(frontLeftPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.rightRear.setPower(backRightPower);
    }

    public void stop() {

    }
}
