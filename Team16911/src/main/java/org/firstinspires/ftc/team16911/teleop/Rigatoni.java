package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@TeleOp(name = "Rigatoni")
public class Rigatoni extends OpMode
{
    RigatoniHardware hardware;
    int maxPosition = 100;
    int currentPosition = 0;
    int lastPosition = -100;
    int armMotorTwoOffset = 0;
    double slowConstant = 1.0;

    boolean justMoved = false;
    boolean newPos = false;
    boolean canRun = false;
    boolean strafeRight = false;
    boolean strafeLeft = false;
    boolean strafingFromJoystick = false;

    double leftFrontPower;
    double leftRearPower;
    double rightFrontPower;
    double rightRearPower;

    ElapsedTime armTime = null;
    ElapsedTime buttonTime = null;
    ElapsedTime rightStrafeTime = null;
    ElapsedTime leftStrafeTime = null;

    public void init()
    {
        // Initialize Hardware
        hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        rightStrafeTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        leftStrafeTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
        //moveArmTwoPointZero();
        spinCarousel();
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

        leftFrontPower = y - x - rx;
        leftRearPower = y + x - rx;
        rightFrontPower = y + x + rx;
        rightRearPower = y - x + rx;

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

        strafe(y, x);

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
        if (gamepad2.right_trigger > .25)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            hardware.armMotorOne.setPower(gamepad2.right_trigger * .7);
            hardware.armMotorTwo.setPower(gamepad2.right_trigger * .7);

            //hardware.armMotorOne.setPower(getUpwardPower(currentPosition));
            //hardware.armMotorTwo.setPower(getUpwardPower(currentPosition));

            justMoved = true;
            newPos = true;
        }
        else if (gamepad2.left_trigger > .25)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            hardware.armMotorOne.setPower(-.1);
            hardware.armMotorTwo.setPower(-.1);

            //hardware.armMotorOne.setPower(getDownwardPower(currentPosition));
            //hardware.armMotorTwo.setPower(getDownwardPower(currentPosition));

            justMoved = true;
            newPos = true;
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

        // Eliminates Arm Oscillations
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

        if (canRun && newPos && armTime.milliseconds() >= 300)
        {
            hardware.armMotorOne.setTargetPosition(currentPosition);
            hardware.armMotorTwo.setTargetPosition(currentPosition + armMotorTwoOffset);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            hardware.armMotorOne.setPower(1);
            hardware.armMotorTwo.setPower(1);

            newPos = false;
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
            newPos = true;
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

    private void moveArmTwoPointZero()
    {
        int armOnePos = hardware.armMotorOne.getCurrentPosition();
        int armTwoPos = hardware.armMotorTwo.getCurrentPosition();

        if (gamepad2.right_trigger > 0 && armTime.time() >= 40)
        {
            hardware.armMotorOne.setTargetPosition(Math.min(armOnePos + 1, 100));
            hardware.armMotorTwo.setTargetPosition(Math.min(armTwoPos + 1, 100));
            armTime.reset();
        }
        else if (gamepad2.left_trigger > 0 && armTime.time() >= 40)
        {
            hardware.armMotorOne.setTargetPosition(Math.max(0, armOnePos - 1));
            hardware.armMotorTwo.setTargetPosition(Math.max(0, armTwoPos - 1));
            armTime.reset();
        }
    }

    private void spinCarousel()
    {
        // Carousel Motor Code
        if (gamepad1.right_bumper)
        {
            hardware.carouselMotorOne.setDirection(DcMotorEx.Direction.FORWARD);
            hardware.carouselMotorOne.setPower(.6);

            hardware.carouselMotorTwo.setDirection(DcMotorEx.Direction.REVERSE);
            hardware.carouselMotorTwo.setPower(.6);
        }
        else if (gamepad1.left_bumper)
        {
            hardware.carouselMotorOne.setDirection(DcMotorEx.Direction.REVERSE);
            hardware.carouselMotorOne.setPower(.6);

            hardware.carouselMotorTwo.setDirection(DcMotorEx.Direction.FORWARD);
            hardware.carouselMotorTwo.setPower(.6);
        }
        else
        {
            hardware.carouselMotorOne.setPower(0.0);
            hardware.carouselMotorTwo.setPower(0.0);
        }
    }

    private void strafe(double y, double x)
    {
        // Strafes from D-Pad Buttons
        if (gamepad1.dpad_up || gamepad1.dpad_right)
        {
            strafeRight();
            strafeLeft = false;
        }
        else if (gamepad1.dpad_down || gamepad1.dpad_left)
        {
            strafeLeft();
            strafeRight = false;
        }
        else if (!strafingFromJoystick)
        {
            strafeRight = false;
            strafeLeft = false;
        }

        // Strafes from joystick
        if (Math.abs(y) <= .1 && x >= .9)
        {
            strafeRight();
            strafeLeft = false;
            strafingFromJoystick = true;
        }
        else if (Math.abs(y) <= .1 && x <= -.9)
        {
            strafeLeft();
            strafeRight = false;
            strafingFromJoystick = true;
        }
        else
        {
            strafingFromJoystick = false;
        }
    }

    private void strafeRight()
    {
        if (!strafeRight)
        {
            rightStrafeTime.reset();
        }

        strafeRight = true;

        if (strafeRight && rightStrafeTime.time() <= 400)
        {
            leftFrontPower = -1;
            leftRearPower = .67;
            rightRearPower = -.67;
            rightFrontPower = 1;
        }
        else if (strafeRight)
        {
            leftFrontPower = -1;
            leftRearPower = 1;
            rightRearPower = -1;
            rightFrontPower = 1;
        }
    }

    private void strafeLeft()
    {
        if (!strafeLeft)
        {
            leftStrafeTime.reset();
        }

        strafeLeft = true;

        if (strafeLeft && leftStrafeTime.time() <= 400)
        {
            leftFrontPower = 1;
            leftRearPower = -.7;
            rightRearPower = .7;
            rightFrontPower = -1;
        }
        else if (strafeLeft)
        {
            leftFrontPower = 1;
            leftRearPower = -1;
            rightRearPower = 1;
            rightFrontPower = -1;
        }
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
