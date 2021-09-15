package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.*;

public class RobotHardware {
    // Primary wheel motors
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx[] wheels = null;
    public List<DcMotorEx> motors = new ArrayList<>();

    // Primary encoders
    public Encoder encoderLeft = null;
    public Encoder encoderRight = null;
    public Encoder encoderFront = null;
    public List<Encoder> encoders = new ArrayList<>();

    // Servos collection
    public List<Servo> servos = new ArrayList<>();

    // Battery voltage sensor
    public VoltageSensor batteryVoltageSensor = null;

    // Hardware map
    public HardwareMap hardwareMap = null;

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    public void init(HardwareMap hardwareMap, boolean shouldInitializeComponents) {
        // Setup hardware map
        Assert.assertNotNull(hardwareMap);
        this.hardwareMap = hardwareMap;

        // Initialize wheels
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, LEFT_REAR_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, RIGHT_REAR_MOTOR);

        // Prepare wheel and motor collections
        wheels = new DcMotorEx[]{leftFront, rightFront, leftRear, rightRear};
//        motors.addAll(Arrays.asList(wheels));

        // Initialize encoders
        encoderLeft = new Encoder(hardwareMap.get(DcMotorEx.class, LEFT_ENCODER));
        encoderRight = new Encoder(hardwareMap.get(DcMotorEx.class, RIGHT_ENCODER));
        encoderFront = new Encoder(hardwareMap.get(DcMotorEx.class, FRONT_ENCODER));

        // Prepare encoder collection
        encoders.addAll(Arrays.asList(encoderLeft, encoderRight, encoderFront));

        // Initialize batter voltage sensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Prepare motor directions
//        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
//        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
//        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
//        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize the components if it is specified
        if (shouldInitializeComponents) {
            initializeComponents();
        }
    }

    public void initializeComponents() {
        // Setup motors
        for (DcMotorEx motor : motors) {
            // Set all motors to zero power
            motor.setPower(0);

            // Motors will break on Zero power
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            // Set all motors to run without encoders.
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Setup servos
        for (Servo servo : servos) {
            // Set all servos to position 0
            servo.setPosition(0);
        }
    }
}
