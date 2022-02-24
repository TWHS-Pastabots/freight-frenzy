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
    SampleMecanumDrive drive = null;

    int currentPosition = 0;
    int lastPosition = -100;
    int armMotorTwoOffset = 0;
    int outtakePowerIndex = 1;
    double slowConstant = .6;
    double[] outtakePowers = {-.3, -.45, -.8};
    boolean usePowerScaling = true;

    boolean justMoved = false;
    boolean canRun = false;

    double leftFrontPower;
    double leftRearPower;
    double rightFrontPower;
    double rightRearPower;

    boolean autoDriveForward = false;
    boolean autoDriveBackward = false;

    final int MAX_AUTO_DRIVE_TIME = 950;

    ElapsedTime armTime = null;
    ElapsedTime buttonTime = null;
    ElapsedTime autoDriveTime = null;
    ElapsedTime carouselTime = null;
    ElapsedTime outtakePowerButtonTime = null;

    public void init()
    {
        // Initialize Hardware
        hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        autoDriveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        outtakePowerButtonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
        turnServo();
        autoTurn();

        telemetry.addData("Using Power Scaling", usePowerScaling);
        telemetry.addData("Current Outtake Power", outtakePowers[outtakePowerIndex]);
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

        autoDrive(x, y, rx);

        hardware.leftFront.setPower(leftFrontPower * (slowConstant + finalSlowConstantOffset));
        hardware.leftRear.setPower(leftRearPower * (slowConstant + finalSlowConstantOffset));
        hardware.rightFront.setPower(rightFrontPower * (slowConstant + finalSlowConstantOffset));
        hardware.rightRear.setPower(rightRearPower * (slowConstant + finalSlowConstantOffset));
    }

    private void autoDrive(double x, double y, double rx)
    {
        rx = rx * .45;

        if (gamepad1.dpad_up)
        {
            autoDriveForward = true;
            autoDriveBackward = false;
            autoDriveTime.reset();
        }
        else if (gamepad1.dpad_down)
        {
            autoDriveForward = false;
            autoDriveBackward = true;
            autoDriveTime.reset();
        }

        if (x != 0.0 || y != 0.0 || gamepad1.right_stick_button)
        {
            autoDriveForward = false;
            autoDriveBackward = false;
        }

        if (autoDriveForward && autoDriveTime.time() < MAX_AUTO_DRIVE_TIME)
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

    private void autoTurn()
    {
        if (gamepad1.triangle) { drive.turn(Math.toRadians(90)); }
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

        // Resets zero position for calibration
        if (gamepad2.triangle)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        /*telemetry.addData("Using Power Scaling", usePowerScaling);
        telemetry.addData("Arm One Pos", hardware.armMotorOne.getCurrentPosition());
        telemetry.addData("Arm One Target", hardware.armMotorOne.getTargetPosition());
        telemetry.addData("Arm Two Pos", hardware.armMotorTwo.getCurrentPosition());
        telemetry.addData("Arm Two Target", hardware.armMotorTwo.getTargetPosition());
        telemetry.addData("Current Position", currentPosition);*/
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
        if (gamepad2.dpad_up && outtakePowerButtonTime.time() > 500)
        {
            outtakePowerIndex = Math.min(outtakePowers.length - 1, outtakePowerIndex + 1);
            outtakePowerButtonTime.reset();
        }
        else if (gamepad2.dpad_down && outtakePowerButtonTime.time() > 500)
        {
            outtakePowerIndex = Math.max(0, outtakePowerIndex - 1);
            outtakePowerButtonTime.reset();
        }
        else if (gamepad2.square && outtakePowerButtonTime.time() > 500)
        {
            outtakePowerIndex = 1;
            outtakePowerButtonTime.reset();
        }

        if (gamepad2.right_bumper)
        {
            hardware.intakeMotor.setPower(.9);
        }
        else if (gamepad2.left_bumper)
        {
            hardware.intakeMotor.setPower(outtakePowers[outtakePowerIndex]);
        }
        else
        {
            hardware.intakeMotor.setPower(0.0);
        }
    }

    private void turnServo() { hardware.cappingServo.setPower(gamepad2.left_stick_y); }

    private double getUpwardPower(int currentPosition)
    {
        return -.0000175 * currentPosition * currentPosition + currentPosition * .0035 + .675;
    }

    private double getDownwardPower(int currentPosition)
    {
        return -.00001 * currentPosition * currentPosition + currentPosition * .002 - .1;
    }
}

