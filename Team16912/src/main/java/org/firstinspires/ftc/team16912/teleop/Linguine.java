package org.firstinspires.ftc.team16912.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16912.util.LinguineHardware;
import org.firstinspires.ftc.team16912.util.Util;

@TeleOp(name = "Linguine")
public class Linguine extends LinearOpMode {

    // Initialize
    LinguineHardware robot = new LinguineHardware();

    ElapsedTime runTime = new ElapsedTime();

    // Variables
    private double speedMult = .75;

    // Checks if running
    private boolean isActive() {
        return !isStopRequested() && opModeIsActive();
    }


    // Runs once on start
    public void runOpMode() {

        robot.init(hardwareMap, true);
        Util.init(hardwareMap);

        waitForStart();


        // Loop
        while (isActive()) {

            // Mecanum drive code
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            double frontLeftPower = y - x - rx;
            double backLeftPower = y + x - rx;
            double frontRightPower = y + x + rx;
            double backRightPower = y - x + rx;

            if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                    Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
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

            robot.motorLeftFront.setPower(speedMult * frontLeftPower);
            robot.motorLeftRear.setPower(speedMult * backLeftPower);
            robot.motorRightFront.setPower(speedMult * frontRightPower);
            robot.motorRightRear.setPower(speedMult * backRightPower);

            if (gamepad1.triangle) speedMult = .3;
            else if (gamepad1.circle) speedMult = 1;
            else if (gamepad1.square) speedMult = .75;


            // Arm up
            if (gamepad2.dpad_up) Util.armUp();

            // Arm down
            else if (gamepad2.dpad_down) Util.armDown();

            // Zero power to the arm
            else Util.armZero();

            // Reset Arm
            if (gamepad2.triangle) Util.runArmTo(0);

            // Carousel Spinner
            if (gamepad2.dpad_right) Util.setSpinnerDirection('f');
            else if (gamepad2.dpad_left) Util.setSpinnerDirection('r');

            // Auto duck end game spinner
            if(gamepad2.cross)
            {
                double time = 0.0;
                for(int i=0; i<10; i++)
                {
                    robot.cSpinner.setVelocity(300);
                    time = runTime.milliseconds();
                    while (runTime.milliseconds() - time < 1700){
                    }
                    robot.cSpinner.setVelocity(0);
                    time = runTime.milliseconds();
                    while (runTime.milliseconds() - time < 500){
                    }
                }
            }

            robot.cSpinner.setVelocity(gamepad2.right_trigger * 200);

            // Claw Activation
            if (gamepad2.square) Util.closeClaw(.01);
            else if (gamepad2.circle) Util.openClaw(.01);

            // Move tape measure up or down
            if(gamepad1.dpad_up) robot.UDMotor.setPower(-.2);
            else if(gamepad1.dpad_down) robot.UDMotor.setPower(.2);
            else robot.UDMotor.setPower(0);

            // Rotate tape measure left or right
            if(gamepad1.dpad_left) robot.LRServo.setPower(-.2);
            else if(gamepad1.dpad_right) robot.LRServo.setPower(.2);
            else robot.LRServo.setPower(0);

            // Send tape in or out
            if (gamepad1.left_trigger > 0) robot.tapeServo.setPower(gamepad1.left_trigger);
            else if (gamepad1.right_trigger > 0) robot.tapeServo.setPower(-gamepad1.right_trigger);
            else robot.tapeServo.setPower(0);



            // Telemetry
            telemetry.addData("Arm Position: ", robot.armEncoder.getCurrentPosition());
            telemetry.update();
        }


    }
}
