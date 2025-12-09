package org.firstinspires.ftc.teamcode.opMode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;

@TeleOp(name="Shooter Test")
public class ShooterTest extends OpMode {
    private Shooter shooter;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, telemetry);
        shooter.init();
        telemetry.addData("Status", "Ready - Press A to spin, B to stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Controls
        if (gamepad1.a) {
            shooter.setTargetRPM(3000, 3000);
        }
        if (gamepad1.b) {
            shooter.stop();
        }

        // Update shooter
        shooter.update();

        // Display info
        telemetry.addData("Controls", "A = Spin | B = Stop");
        telemetry.addData("At Speed?", shooter.atTargetSpeed());
        telemetry.update();
    }
}