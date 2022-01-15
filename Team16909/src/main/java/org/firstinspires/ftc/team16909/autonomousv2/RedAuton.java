package org.firstinspires.ftc.team16909.autonomousv2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;

import java.util.ArrayList;
import java.lang.Thread;

@Autonomous(preselectTeleOp = "FettuccineRRv2")
public class RedAuton {

    FettuccineHardware robot = null;
    SampleMecanumDrive drive = null;
    Actions action = null;

    // "Ok" Button
    private boolean ok = false;

    // LISTS
    private ArrayList<String> startChoices = new ArrayList<String>() {{
       add("RedC");
       add("RedW");
    }};
    private ArrayList<String> blockChoices = new ArrayList<String>() {{
        add("RedPushback");
        add("RedHub");
    }};

    public void waitFor (double time) {

        ElapsedTime duration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = duration.seconds();

        while (duration.seconds() - startTime < time) {

        }
        return;
    }
}
