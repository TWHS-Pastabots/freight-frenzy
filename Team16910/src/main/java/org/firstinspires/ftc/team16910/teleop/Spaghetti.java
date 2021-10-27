package org.firstinspires.ftc.team16910.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Spaghetti")
public class Spaghetti extends OpMode {

    SpaghettiHardware robot;
    int maxPosition = 100;
    boolean justMoved = false;

    public void init()
    {
        // Initialize Hardware
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start()
    {
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    public void loop()
    {
        drive();
        //moveArm();
        //spinCarousel();
    }

    public void stop()
    {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private void drive()
    {
        // Mecanum drivecode
        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double leftFrontPower = y - x - rx;
        double leftRearPower = y + x - rx;
        double rightFrontPower = y + x + rx;
        double rightRearPower = y - x + rx;

        if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 ||
                Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1 )
        {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
            max = Math.max(Math.abs(rightFrontPower), max);
            max = Math.max(Math.abs(rightRearPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        if (gamepad1.dpad_up || gamepad1.dpad_right)
        {
            leftFrontPower = -1;
            rightRearPower = -1;
            rightFrontPower = 1;
            leftRearPower = 1;
        }
        else if (gamepad1.dpad_down || gamepad1.dpad_left)
        {
            leftFrontPower = 1;
            rightRearPower = 1;
            rightFrontPower = -1;
            leftRearPower = -1;
        }

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
    }

    private void moveArm()
    {
        // Runs driver controlled code
        if (gamepad1.right_trigger > 0)
        {
            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            robot.armMotorOne.setPower(gamepad1.right_trigger * .5);
            robot.armMotorTwo.setPower(gamepad1.right_trigger * .5);
            justMoved = true;
        }
        else if (gamepad1.left_trigger > 0)
        {
            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            robot.armMotorOne.setPower(getDownwardPower(robot.armMotorOne.getCurrentPosition()));
            robot.armMotorTwo.setPower(getDownwardPower(robot.armMotorTwo.getCurrentPosition()));

            justMoved = true;
        }
        else if (justMoved)
        {
            robot.armMotorOne.setTargetPosition(robot.armMotorOne.getCurrentPosition());
            robot.armMotorTwo.setTargetPosition(robot.armMotorTwo.getCurrentPosition());

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.armMotorOne.setPower(1.0);
            robot.armMotorTwo.setPower(1.0);

            justMoved = false;
        }

        // Runs to highest position
        if (gamepad1.triangle)
        {
            robot.armMotorOne.setTargetPosition(maxPosition);
            robot.armMotorTwo.setTargetPosition(maxPosition);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.armMotorOne.setPower(.5);
            robot.armMotorTwo.setPower(.5);
            justMoved = false;
        }

        // Resets zero position for calibration
        if (gamepad1.dpad_down)
        {
            robot.armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Displays current position for development purpose
        if (gamepad1.circle)
        {
            telemetry.addData("Current Position", robot.armMotorOne.getCurrentPosition());
            telemetry.update();
        }
        else if (gamepad1.square)
        {
            telemetry.addData("Current Position", robot.armMotorTwo.getCurrentPosition());
            telemetry.update();
        }
    }

    private double getUpwardPower(int currentPosition)
    {
        return -.00006 * currentPosition * currentPosition + currentPosition * .006 + .35;
    }

    private double getDownwardPower(int currentPosition) {
        return -.000026 * currentPosition * currentPosition + currentPosition * .0034 - .08;
    }
}
