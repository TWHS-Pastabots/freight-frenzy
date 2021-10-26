package org.firstinspires.ftc.team16912.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team16912.util.LinguineHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Linguine")
public class Linguine extends LinearOpMode {

    // Initialize

    // Checks if running
    private boolean isActive() {
        return !isStopRequested() && opModeIsActive();
    }

    // Converts encoder readings to radians
    private double encToRad(int encVal) { return Math.abs(Math.toRadians(encVal * (145.0 / 112.0))); }


    // Runs once on start
    public void runOpMode() {

        LinguineHardware robot = new LinguineHardware();
        robot.init(hardwareMap, true);

        waitForStart();

        // Init

        // Loop
        while (isActive()) {
            for(DcMotorEx motor : robot.motorArms)
            {
                motor.setPower(0);
            }
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

            if (gamepad2.dpad_up)
            {
                for (DcMotorEx motor : robot.motorArms)
                {
                    if (encToRad(motor.getCurrentPosition()) <= Math.toRadians(90))
                    {
                        motor.setPower(-.8 * Math.cos(encToRad(motor.getCurrentPosition())) - .3);
                        motor.setPower(-1);
                    }

                    else if (encToRad(motor.getCurrentPosition()) >= 0)
                        motor.setPower(-1);
                }
            }

            else if (gamepad2.dpad_down)
            {
                for (DcMotorEx motor : robot.motorArms)
                {
                    if (encToRad(motor.getCurrentPosition()) <= Math.toRadians(90))
                    {
                        motor.setPower(.2 * Math.cos(encToRad(motor.getCurrentPosition())) + .3);
                        motor.setPower(-1);
                    }


                    else if (encToRad(motor.getCurrentPosition()) >= 0) {
                        motor.setPower(-1);
                    }
                }
            }

            if (gamepad2.left_bumper) {
                robot.cSpinner.setVelocity(3000);
            }

            else if (gamepad2.right_bumper) {
                robot.cSpinner.setVelocity(-3000);
            }

            else robot.cSpinner.setVelocity(0);

            telemetry.addData("Arm Position: ", encToRad(robot.motorArm1.getCurrentPosition()));
            telemetry.addData("Motor Arm 1: ", robot.motorArm1.getDirection());
            telemetry.addData("Motor Arm 2: ", robot.motorArm2.getDirection());
            telemetry.update();
        }
    }
}
