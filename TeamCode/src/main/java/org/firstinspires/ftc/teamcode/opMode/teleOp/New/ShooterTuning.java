package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.New.Shooter2;

@TeleOp(name="Shooter Tuning", group = " Tuning")
public class ShooterTuning extends OpMode {
    public Shooter2 shooter;

    private double manualShooterRPM = 0;
    private double manualCRRPM = 0;
    private boolean manualMode = false;

    @Override
    public void init() {
        shooter = new Shooter2(hardwareMap, telemetry);
        shooter.init();

        telemetry.addData("Status", "Ready - Use FTC Dashboard to tune PID");
        telemetry.addData("Manual Mode", "Dpad Up/Down for Shooter, Left/Right for CR");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleManualControls();

        shooter.update();

        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Close Preset");
        telemetry.addData("Y", "Far Preset");
        telemetry.addData("X", "Manual Mode Toggle");
        telemetry.addData("B", "Stop");
        telemetry.addData("", "");

        if (manualMode) {
            telemetry.addData("=== MANUAL MODE ===", "ACTIVE");
            telemetry.addData("Dpad Up/Down", "Shooter RPM ±100");
            telemetry.addData("Dpad Left/Right", "Counter Roller RPM ±100");
            telemetry.addData("Manual Shooter RPM", manualShooterRPM);
            telemetry.addData("Manual CR RPM", manualCRRPM);
        } else {
            telemetry.addData("Mode", "Preset");
        }

        telemetry.addData("", "");
        telemetry.addData("=== SHOOTER STATUS ===", "");
        telemetry.addData("State", shooter.getState());
        telemetry.addData("At Speed", shooter.atTargetSpeed());
        telemetry.addData("", "");

        telemetry.addData("=== SHOOTER MOTOR ===", "");
        telemetry.addData("Current RPM", String.format("%.0f", shooter.getShooterRPM()));
        telemetry.addData("Target RPM", String.format("%.0f", getShooterTargetRPM()));
        telemetry.addData("Error", String.format("%.0f", getShooterTargetRPM() - shooter.getShooterRPM()));
        telemetry.addData("", "");

        telemetry.addData("=== COUNTER ROLLER ===", "");
        telemetry.addData("Current RPM", String.format("%.0f", shooter.getCounterRollerRPM()));
        telemetry.addData("Target RPM", String.format("%.0f", getCRTargetRPM()));
        telemetry.addData("Error", String.format("%.0f", getCRTargetRPM() - shooter.getCounterRollerRPM()));
        telemetry.addData("", "");

        telemetry.addData("=== TUNE IN FTC DASHBOARD ===", "");
        telemetry.addData("Shooter", "KP, KI, KD, KF");
        telemetry.addData("Counter Roller", "CR_KP, CR_KI, CR_KD, CR_KF");
        telemetry.addData("Presets", "CLOSE_SHOOTER_RPM, CLOSE_CR_RPM");
        telemetry.addData("", "FAR_SHOOTER_RPM, FAR_CR_RPM");

        telemetry.update();
    }

    private void handleManualControls() {
        if (gamepad1.x) {
            manualMode = !manualMode;
            sleep(200);
        }

        if (manualMode) {
            if (gamepad1.dpad_up) {
                manualShooterRPM += 100;
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                manualShooterRPM -= 100;
                manualShooterRPM = Math.max(0, manualShooterRPM);
                sleep(100);
            }
            if (gamepad1.dpad_right) {
                manualCRRPM += 100;
                sleep(100);
            }
            if (gamepad1.dpad_left) {
                manualCRRPM -= 100;
                manualCRRPM = Math.max(0, manualCRRPM);
                sleep(100);
            }

            if (gamepad1.a) {
                setManualSpeed(manualShooterRPM, manualCRRPM);
            }

            if (gamepad1.b) {
                shooter.stop();
                manualShooterRPM = 0;
                manualCRRPM = 0;
            }
        } else {
            shooter.updateCtrls(gamepad1, gamepad2);
        }
    }

    private void setManualSpeed(double shooterRPM, double crRPM) {
        Shooter2.CLOSE_SHOOTER_RPM = shooterRPM;
        Shooter2.CLOSE_CR_RPM = crRPM;
        shooter.spinUpClose();
    }

    private double getShooterTargetRPM() {
        switch (shooter.getState()) {
            case READY_CLOSE:
            case SPINNING_UP_CLOSE:
                return Shooter2.CLOSE_SHOOTER_RPM;
            case READY_FAR:
            case SPINNING_UP_FAR:
                return Shooter2.FAR_SHOOTER_RPM;
            default:
                return 0;
        }
    }

    private double getCRTargetRPM() {
        switch (shooter.getState()) {
            case READY_CLOSE:
            case SPINNING_UP_CLOSE:
                return Shooter2.CLOSE_CR_RPM;
            case READY_FAR:
            case SPINNING_UP_FAR:
                return Shooter2.FAR_CR_RPM;
            default:
                return 0;
        }
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}