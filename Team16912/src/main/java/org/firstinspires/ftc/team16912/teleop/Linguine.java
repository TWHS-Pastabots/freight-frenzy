package org.firstinspires.ftc.team16912.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team16912.util.LinguineHardware;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "Linguine")
public class Linguine extends LinearOpMode {

    // Initialize
    LinguineHardware robot = new LinguineHardware();

    // Global Variables
    private double speedMult = .75;

    // Checks if running
    private boolean isActive() {
        return !isStopRequested() && opModeIsActive();
    }


    // Runs once on start
    public void runOpMode() {

        robot.init(hardwareMap, true);

        waitForStart();

        // Init
        robot.motorArm1.setTargetPositionTolerance(5);
        robot.motorArm2.setTargetPositionTolerance(5);

        // Loop
        while (isActive()) {
            for(DcMotorEx motor : robot.motorArms)
            {
                motor.setPower(0);
            }
            // Mecanum drivecode
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x -  rx;
            double backLeftPower = y + x + rx;
            double frontRightPower = y - x + rx;
            double backRightPower = y - x - rx;

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

            robot.motorLeftFront.setPower(speedMult * frontLeftPower);
            robot.motorLeftRear.setPower(speedMult * backLeftPower);
            robot.motorRightFront.setPower(speedMult * frontRightPower);
            robot.motorRightRear.setPower(speedMult * backRightPower);

            if (gamepad1.triangle) speedMult = .3;
            else if (gamepad1.circle) speedMult = 1;
            else if (gamepad1.square) speedMult = .75;


            if (gamepad2.dpad_up) {
                //decrease arm power so arm does not
                if (robot.armEncoder.getCurrentPosition() < 3000 && robot.armEncoder.getCurrentPosition() > 1200)
                    for (DcMotorEx motor : robot.motorArms) motor.setPower(-.4);
                else for (DcMotorEx motor : robot.motorArms) motor.setPower(-.5);

            }
            else if (gamepad2.dpad_down) {
                if (robot.armEncoder.getCurrentPosition() < 2340) {
                    for (DcMotorEx motor : robot.motorArms) motor.setPower(.1);
                    if (robot.armEncoder.getCurrentPosition() < 1200)
                        for (DcMotorEx motor : robot.motorArms) motor.setPower(0 );
                }

                else for (DcMotorEx motor : robot.motorArms) motor.setPower(.5);
            }
//            if (gamepad2.dpad_up)
//            {
//                telemetry.addLine("UP");
//                if (encToRad(robot.armEncoder.getCurrentPosition()) > -100 && encToRad(robot.armEncoder.getCurrentPosition()) < Math.PI / 2) {
//                    robot.motorArm1.setPower(-.4 * Math.cos(encToRad(robot.armEncoder.getCurrentPosition()) - .6));
//                    robot.motorArm2.setPower(-.4 * Math.cos(encToRad(robot.armEncoder.getCurrentPosition()) - .6));
//                }
//
//                else if (encToRad(robot.armEncoder.getCurrentPosition()) >= Math.PI / 2) {
//                    robot.motorArm1.setPower(-.2);
//                    robot.motorArm2.setPower(-.2);
//                }
//            }
//
//            else if (gamepad2.dpad_down)
//            {
//                telemetry.addLine("DOWN");
//                if (encToRad(robot.armEncoder.getCurrentPosition()) >= Math.PI / 2) {
//                    robot.motorArm1.setPower(.4 * Math.cos(encToRad(robot.armEncoder.getCurrentPosition()) + .6));
//                    robot.motorArm2.setPower(.4 * Math.cos(encToRad(robot.armEncoder.getCurrentPosition()) + .6));
//                }
//
//                else if (encToRad(robot.armEncoder.getCurrentPosition()) > -100 && encToRad(robot.armEncoder.getCurrentPosition()) < Math.PI / 2) {
//                    robot.motorArm1.setPower(.2);
//                    robot.motorArm2.setPower(.2);
//                }
//            }

            else {
                robot.motorArm1.setPower(0);
                robot.motorArm2.setPower(0);
            }

            if (gamepad2.dpad_right) robot.cSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
            else if (gamepad2.dpad_left) robot.cSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.cSpinner.setVelocity(gamepad2.right_trigger * 1000);

            // Claw Activation
            if (gamepad2.square) {
                robot.servoClawLeft.setPosition(1);
                robot.servoClawRight.setPosition(1);
            }

            else if (gamepad2.circle) {
                robot.servoClawLeft.setPosition(0);
                robot.servoClawRight.setPosition(0);
            }

            // Telemetry
            telemetry.addData("Arm Position: ", robot.armEncoder.getCurrentPosition());
            telemetry.addData("Motor Arm 1: ", robot.motorArm1.getDirection());
            telemetry.addData("Motor Arm 2: ", robot.motorArm2.getDirection());
            telemetry.update();
        }


    }

    // Converts encoder readings to radians
    // Rad = tick * pi / 4100
    private double encToRad(int encVal) { return Math.abs(encVal * (Math.PI / 4100) ); }



}
