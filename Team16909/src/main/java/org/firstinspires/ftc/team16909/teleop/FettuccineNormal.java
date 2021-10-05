package org.firstinspires.ftc.team16909.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class FettuccineNormal extends LinearOpMode
{
    // Checks if running
    private boolean isActive() {
        return !isStopRequested() && opModeIsActive();
    }


    // Runs once on start
    public void runOpMode() {

        RobotHardware robot = new RobotHardware();
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
}
