package org.firstinspires.ftc.team15021.hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RavioliHardware
{
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx[] motors;

    public void init(HardwareMap hardwareMap)
    {
        initializeDriveMotors(hardwareMap);
    }

    private void initializeDriveMotors(HardwareMap hardwareMap)
    {
        leftFront = hardwareMap.get(DcMotorEx.class, RavioliIds.LEFT_FRONT_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, RavioliIds.RIGHT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, RavioliIds.LEFT_REAR_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, RavioliIds.RIGHT_REAR_MOTOR);

        motors = new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear};

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        for (DcMotorEx motor : motors)
        {
            motor.setPower(0.0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }
}
