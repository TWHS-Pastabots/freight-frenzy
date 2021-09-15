package org.firstinspires.ftc.team15021.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Ravioli")
public class Ravioli extends LinearOpMode {

    // Initialization
    RobotHardware robot;

    // Run Linear
    public void runOpMode() {

        // Init robot hardware devices
        robot = new RobotHardware();
        robot.init(hardwareMap, true);


        // Loop
        while (isActive()) {

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

            robot.motorLeftFront.setPower(frontLeftPower);
            robot.motorLeftRear.setPower(backLeftPower);
            robot.motorRightFront.setPower(frontRightPower);
            robot.motorRightRear.setPower(backRightPower);
        }
    }


    // Assistance functions

    private boolean isActive() {
        return !isStopRequested() && opModeIsActive();
    }

}
