package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@TeleOp(name = "Rigatoni")
public class Rigatoni extends OpMode
{
    RigatoniHardware hardware = null;

    int maxPosition = 230;
    int currentPosition = 0;
    int lastPosition = -100;
    int armMotorTwoOffset = 0;
    double slowConstant = .6;
    boolean usePowerScaling = true;

    boolean justMoved = false;
    boolean canRun = false;

    double leftFrontPower;
    double leftRearPower;
    double rightFrontPower;
    double rightRearPower;

    boolean autoStrafeRight = false;
    boolean autoStrafeLeft = false;
    boolean autoDriveForward = false;
    boolean autoDriveBackward = false;

    final int MAX_AUTO_STRAFE_TIME = 1350;
    final int MAX_AUTO_DRIVE_TIME = 950;

    ElapsedTime armTime = null;
    ElapsedTime buttonTime = null;
    ElapsedTime autoDriveTime = null;
    ElapsedTime carouselTime = null;

    public void init()
    {
        // Initialize Hardware
        hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        autoDriveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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

        if (gamepad2.cross && buttonTime.time() > 500 && usePowerScaling)
        {
            usePowerScaling = false;
            buttonTime.reset();
        }
        else if (gamepad2.cross && buttonTime.time() > 500 && !usePowerScaling)
        {
            usePowerScaling = true;
            buttonTime.reset();
        }

        moveArm();
        spinCarousel();
        operateIntake();
        //turnServo();

        telemetry.update();
    }

    public void stop()
    {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private void drive()
    {
        // Mecanum drivecode
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;

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

        double slowConstantPositiveOffset = (1 - slowConstant) * gamepad1.right_trigger;
        slowConstantPositiveOffset -= slowConstantPositiveOffset * Math.abs(gamepad1.left_stick_y);

        double slowConstantNegativeOffset = -.4 * gamepad1.left_trigger;

        double finalSlowConstantOffset = slowConstantPositiveOffset + slowConstantNegativeOffset;

        turn();

        autoDrive(x, y, rx);

        hardware.leftFront.setPower(leftFrontPower * (slowConstant + finalSlowConstantOffset));
        hardware.leftRear.setPower(leftRearPower * (slowConstant + finalSlowConstantOffset));
        hardware.rightFront.setPower(rightFrontPower * (slowConstant + finalSlowConstantOffset));
        hardware.rightRear.setPower(rightRearPower * (slowConstant + finalSlowConstantOffset));
    }

    private void autoDrive(double x, double y, double rx)
    {
        rx = rx * .45;

        if (gamepad1.dpad_right)
        {
            autoStrafeRight = true;
            autoStrafeLeft = false;
            autoDriveForward = false;
            autoDriveBackward = false;
            autoDriveTime.reset();
        }
        else if (gamepad1.dpad_left)
        {
            autoStrafeRight = false;
            autoStrafeLeft = true;
            autoDriveForward = false;
            autoDriveBackward = false;
            autoDriveTime.reset();
        }
        else if (gamepad1.dpad_up)
        {
            autoStrafeRight = false;
            autoStrafeLeft = false;
            autoDriveForward = true;
            autoDriveBackward = false;
            autoDriveTime.reset();
        }
        else if (gamepad1.dpad_down)
        {
            autoStrafeRight = false;
            autoStrafeLeft = false;
            autoDriveForward = false;
            autoDriveBackward = true;
            autoDriveTime.reset();
        }

        if (x != 0.0 || y != 0.0 || gamepad1.right_stick_button)
        {
            autoStrafeRight = false;
            autoStrafeLeft = false;
            autoDriveForward = false;
            autoDriveBackward = false;
        }

        if (autoStrafeRight && autoDriveTime.time() < MAX_AUTO_STRAFE_TIME)
        {
            leftFrontPower = 1 - rx;
            leftRearPower = -1 - rx;
            rightFrontPower = -1 + rx;
            rightRearPower = 1 + rx;
        }
        else if (autoStrafeLeft && autoDriveTime.time() < MAX_AUTO_STRAFE_TIME)
        {
            leftFrontPower = -1 - rx;
            leftRearPower = 1 - rx;
            rightFrontPower = 1 + rx;
            rightRearPower = -1 + rx;
        }
        else if (autoDriveForward && autoDriveTime.time() < MAX_AUTO_DRIVE_TIME)
        {
            double power = .85;
            leftFrontPower = power - rx;
            leftRearPower = power - rx;
            rightFrontPower = power + rx;
            rightRearPower = power + rx;
        }
        else if (autoDriveBackward && autoDriveTime.time() < MAX_AUTO_DRIVE_TIME)
        {
            double power = -.85;
            leftFrontPower = power - rx;
            leftRearPower = power - rx;
            rightFrontPower = power + rx;
            rightRearPower = power + rx;
        }
    }

    private void turn()
    {
        if (gamepad1.right_bumper)
        {
            leftFrontPower = -.2;
            leftRearPower = -.2;
            rightRearPower = .2;
            rightFrontPower = .2;
        }
        else if (gamepad1.left_bumper)
        {
            leftFrontPower = .2;
            leftRearPower = .2;
            rightRearPower = -.2;
            rightFrontPower = -.2;
        }
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

            if (usePowerScaling)
            {
                hardware.armMotorOne.setPower(getUpwardPower(currentPosition));
                hardware.armMotorTwo.setPower(getUpwardPower(currentPosition));
            }
            else
            {
                hardware.armMotorOne.setPower(gamepad2.right_trigger * .935);
                hardware.armMotorTwo.setPower(gamepad2.right_trigger * .935);
            }

            justMoved = true;
        }
        else if (gamepad2.left_trigger > 0)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            if (usePowerScaling)
            {
                hardware.armMotorOne.setPower(getDownwardPower(currentPosition));
                hardware.armMotorTwo.setPower(getDownwardPower(currentPosition));
            }
            else
            {
                hardware.armMotorOne.setPower(gamepad2.left_trigger * -.2);
                hardware.armMotorTwo.setPower(gamepad2.left_trigger * -.2);
            }

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

        if (canRun && armTime.milliseconds() >= 300)
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

            hardware.armMotorOne.setPower(.8);
            hardware.armMotorTwo.setPower(.8);
        }

        // Resets zero position for calibration
        if (gamepad2.dpad_down)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Using Power Scaling", usePowerScaling);
        telemetry.addData("Arm One Pos", hardware.armMotorOne.getCurrentPosition());
        telemetry.addData("Arm One Target", hardware.armMotorOne.getTargetPosition());
        telemetry.addData("Arm Two Pos", hardware.armMotorTwo.getCurrentPosition());
        telemetry.addData("Arm Two Target", hardware.armMotorTwo.getTargetPosition());
        telemetry.addData("Current Position", currentPosition);
    }


    private void spinCarousel()
    {
        // Carousel Motor Code
        if (gamepad1.circle)
        {
            carouselTime.reset();
            hardware.carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (gamepad1.square)
        {
            carouselTime.reset();
            hardware.carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (carouselTime.time() <= 1200)
        {
            double power = .0002593 * carouselTime.time() + .625;
            hardware.carouselMotor.setPower(power);
        }
        else
        {
            hardware.carouselMotor.setPower(0);
        }
    }

    private void operateIntake()
    {
        if (gamepad2.right_bumper)
        {
            hardware.intakeMotor.setPower(.9);
        }
        else if (gamepad2.left_bumper)
        {
            hardware.intakeMotor.setPower(-.55);
        }
        else
        {
            hardware.intakeMotor.setPower(0.0);
        }
    }

    private void turnServo()
    {
        hardware.cappingServo.setPower(gamepad2.right_stick_y);
    }

    private double getUpwardPower(int currentPosition)
    {
        return -.0000175 * currentPosition * currentPosition + currentPosition * .0035 + .675;
    }

    private double getDownwardPower(int currentPosition)
    {
        return -.00001 * currentPosition * currentPosition + currentPosition * .002 - .1;
    }
}
