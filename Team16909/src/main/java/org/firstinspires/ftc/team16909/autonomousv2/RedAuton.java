package org.firstinspires.ftc.team16909.autonomousv2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16909.autonomousv2.blocks.RedBlocks;
import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;

import java.util.ArrayList;
import java.lang.Thread;

@Autonomous(preselectTeleOp = "FettuccineRRv2")
public class RedAuton extends LinearOpMode {

    FettuccineHardware robot = null;
    SampleMecanumDrive drive = null;
    Actions actions = null;

    // "Ok" Button
    private boolean ok = false;
    private boolean ok2 = false;

    // LISTS
    private ArrayList<String> startChoices = new ArrayList<String>() {{
       add("RedC");
       add("RedW");
    }};
    private ArrayList<String> blockChoices = new ArrayList<String>() {{
        add("RedPushback");
        add("RedHub");
        add("RedCarousel");
        add("RedUnit");
        add("RedWarehouse");
    }};

    private ArrayList<String> sequence = new ArrayList<String>();

    public void runOpMode() throws InterruptedException {

        robot = new FettuccineHardware();
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        actions = new Actions(robot);

       RedBlocks  redBlocks = new RedBlocks(drive, robot, actions);

       int i = 0;
       while (!ok) {
           if (gamepad1.cross) ok = true;
           telemetry.addData("Start", startChoices.get(i));
           if (gamepad1.dpad_right) {
               if (i < startChoices.size() - 1) i++;
               else i = 0;
               waitFor(0.1);
           } else if (gamepad1.dpad_left) {
               if (i>0) i--;
               else i = startChoices.size() - 1;
               waitFor(0.1);
           }
           telemetry.update();
       }
       waitFor(0.1);
       redBlocks.getStart(startChoices.get(i));

        i = 0;
        while (!ok) {
            while (!ok2) {
                if (gamepad1.circle) ok2 = true;
                telemetry.addData("Start", startChoices.get(i));
                if (gamepad1.dpad_right) {
                    if (i < startChoices.size() - 1) i++;
                    else i = 0;
                    waitFor(0.1);
                } else if (gamepad1.dpad_left) {
                    if (i>0) i--;
                    else i = startChoices.size() - 1;
                    waitFor(0.1);
                }
            }
            waitFor(0.1);
            ok2 = false;
        }
        waitFor(0.1);
        redBlocks.getStart(startChoices.get(i));

    }

    public void waitFor (double time) {

        ElapsedTime duration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = duration.seconds();

        while (duration.seconds() - startTime < time) {

        }
        return;
    }
}
