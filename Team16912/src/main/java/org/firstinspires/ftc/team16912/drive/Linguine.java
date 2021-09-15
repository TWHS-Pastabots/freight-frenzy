package org.firstinspires.ftc.team16912.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Linguine")
public class Linguine extends LinearOpMode {

    // Initialize

    // Checks if running
    private boolean isActive() {
        return !isStopRequested() && opModeIsActive();
    }


    // Runs once on start
    public void runOpMode() {


        // Loop
        while (isActive()) {

        }
    }
}
