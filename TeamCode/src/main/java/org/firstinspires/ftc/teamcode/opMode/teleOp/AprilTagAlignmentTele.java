package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
// Import your other subsystems like Drivetrain, Outtake, etc.

@TeleOp(name = "AprilTagAlignmentTele", group = " ")
public class AprilTagAlignmentTele extends LinearOpMode {

    public AprilTagAlignment alignment;
    public Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        alignment = new AprilTagAlignment(hardwareMap, telemetry);
        alignment.init();
        drivetrain.init();


        waitForStart();

        while (opModeIsActive()) {
            // Update all subsystems
            alignment.updateCtrls(gamepad1, gamepad2);
            drivetrain.updateCtrls(gamepad1, gamepad2);
            alignment.update();
            drivetrain.update();


            telemetry.update();
        }

        alignment.close();
    }
}