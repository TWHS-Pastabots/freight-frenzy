package org.firstinspires.ftc.team15021.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

@TeleOp(name = "Ravioli")
public class Ravioli extends OpMode
{

    // Initialization
    RavioliHardware hardware;
    int maxPosition = 100;
    int currentPosition = 0;
    int lastPosition = -100;
    int armMotorTwoOffset = 0;
    final double FAST_SPEED = .8;
    final double SLOW_SPEED = .5;
    double slowConstant = FAST_SPEED;
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
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double leftFrontPower = y + x + rx;
        double leftRearPower = y - x + rx;
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

        if (gamepad1.square && slowConstant == FAST_SPEED && buttonTime.time() >= 500)
        {
            slowConstant = SLOW_SPEED;
            buttonTime.reset();
        }
        else if (gamepad1.square && slowConstant == SLOW_SPEED && buttonTime.time() >= 500)
        {
            slowConstant = FAST_SPEED;
            buttonTime.reset();
        }

        hardware.leftFront.setPower(leftFrontPower * slowConstant);
        hardware.leftRear.setPower(leftRearPower * slowConstant);
        hardware.rightFront.setPower(rightFrontPower * slowConstant);
        hardware.rightRear.setPower(rightRearPower * slowConstant);
    }

    private void moveArm()
    {
        hardware.armMotorOne.setPower(gamepad2.right_trigger);
        hardware.armMotorTwo.setPower(gamepad2.right_trigger);

        hardware.armMotorOne.setPower(-gamepad2.left_trigger);
        hardware.armMotorTwo.setPower(-gamepad2.left_trigger);

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
        if (gamepad2.left_bumper)
        {
            hardware.servoOne.setPosition(1);
            telemetry.addData("Status", "Moved Servo");
            telemetry.update();
        }
        else if (gamepad2.right_bumper)
        {
            hardware.servoOne.setPosition(0);
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
