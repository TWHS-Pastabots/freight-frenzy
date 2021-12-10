package org.firstinspires.ftc.team15021.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team15021.hardware.RavioliHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Ravioli")
public class Ravioli extends OpMode
{

    // Initialization
    RavioliHardware hardware;
    int maxPosition = 100;
    int currentPosition = 0;
    int lastPosition = -100;
    int armMotorTwoOffset = 0;
    double slowConstant = 1.0;
    boolean justMoved = false;
    boolean canRun = false;
    ElapsedTime armTime = null;
    ElapsedTime buttonTime = null;

    public void init()
    {
        // Initialize Hardware
        hardware = new RavioliHardware();
        hardware.init(hardwareMap);
        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
        moveArm();
        controlClaw();
        spinCarousel();
    }

    private void drive()
    {
        // Mecanum drivecode
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double leftFrontPower = y - x + rx;
        double leftRearPower = y + x + rx;
        double rightFrontPower = y - x - rx;
        double rightRearPower = y + x - rx;

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

        if (gamepad1.square && slowConstant == 1.0 && buttonTime.time() >= 500)
        {
            slowConstant = .5;
            buttonTime.reset();
        }
        else if (gamepad1.square && slowConstant == 0.5 && buttonTime.time() >= 500)
        {
            slowConstant = 1.0;
            buttonTime.reset();
        }

        hardware.leftFront.setPower(leftFrontPower * slowConstant);
        hardware.leftRear.setPower(leftRearPower * slowConstant);
        hardware.rightFront.setPower(rightFrontPower * slowConstant);
        hardware.rightRear.setPower(rightRearPower * slowConstant);
    }

    private void moveArm()
    {
        currentPosition = hardware.armMotorOne.getCurrentPosition();
        armMotorTwoOffset = hardware.armMotorTwo.getCurrentPosition() - currentPosition;

        // Runs driver controlled code
        if (gamepad2.right_trigger > 0)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            hardware.armMotorOne.setPower(gamepad2.right_trigger * .75);
            hardware.armMotorTwo.setPower(gamepad2.right_trigger * .75);

            justMoved = true;
        }
        else if (gamepad2.left_trigger > 0)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            hardware.armMotorOne.setPower(gamepad2.left_trigger * -.75);
            hardware.armMotorTwo.setPower(gamepad2.left_trigger * -.75);

            justMoved = true;
        }
        else if (justMoved)
        {
            hardware.armMotorOne.setTargetPosition(currentPosition);
            hardware.armMotorTwo.setTargetPosition(currentPosition + armMotorTwoOffset);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            hardware.armMotorOne.setPower(1);
            hardware.armMotorTwo.setPower(1);

            justMoved = false;
        }

        if (currentPosition == lastPosition)
        {
            if (!canRun)
            {
                armTime.reset();
            }
            canRun = true;
        }
        else
        {
            lastPosition = currentPosition;
            canRun = false;
        }

        if (canRun && currentPosition == lastPosition && armTime.milliseconds() >= 150)
        {
            hardware.armMotorOne.setTargetPosition(currentPosition);
            hardware.armMotorTwo.setTargetPosition(currentPosition + armMotorTwoOffset);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            hardware.armMotorOne.setPower(1);
            hardware.armMotorTwo.setPower(1);
        }

        // Runs to highest position
        if (gamepad2.triangle)
        {
            hardware.armMotorOne.setTargetPosition(maxPosition);
            hardware.armMotorTwo.setTargetPosition(maxPosition + armMotorTwoOffset);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            hardware.armMotorOne.setPower(.7);
            hardware.armMotorTwo.setPower(.7);

            justMoved = false;
        }

        // Resets zero position for calibration
        if (gamepad2.dpad_down)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Arm One Pos", hardware.armMotorOne.getCurrentPosition());
        telemetry.addData("Arm One Target", hardware.armMotorOne.getTargetPosition());
        telemetry.addData("Arm Two Pos", hardware.armMotorTwo.getCurrentPosition());
        telemetry.addData("Arm Two Target", hardware.armMotorTwo.getTargetPosition());
        telemetry.addData("Current Position", currentPosition);
        telemetry.update();
    }
    private void spinCarousel()
    {
        // Carousel Motor Code
        if (gamepad1.right_trigger > 0)
        {
            hardware.carouselMotorOne.setPower(gamepad1.right_trigger * .5);
        }
        else
        {
            hardware.carouselMotorOne.setPower(-gamepad1.left_trigger * .5);
        }
    }

    private void controlClaw()
    {
        if (gamepad2.right_bumper)
        {
            hardware.servoOne.setPosition(1);
            hardware.servoTwo.setPosition(1);
        }
        else if (gamepad2.left_bumper)
        {
            hardware.servoOne.setPosition(-1);
            hardware.servoTwo.setPosition(-1);
        }
    }

    public void stop()
    {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private double getUpwardPower(int currentPosition)
    {
        return -.00012 * currentPosition * currentPosition + currentPosition * .012 + .35;
    }

    private double getDownwardPower(int currentPosition)
    {
        return -.000033 * currentPosition * currentPosition + currentPosition * .0033 - .075;
    }
}
