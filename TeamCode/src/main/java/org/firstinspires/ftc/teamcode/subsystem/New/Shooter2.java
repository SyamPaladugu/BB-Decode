package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Shooter2 implements Subsystem {
    public static double KP = 0.01;
    public static double KI = 0.0;
    public static double KD = 0.0;

    public static double CR_KP = 0.01;
    public static double CR_KI = 0.0;
    public static double CR_KD = 0.0;

    public static double TICKS_PER_REV = 28.0;
    public static double CR_TICKS_PER_REV = 28.0;
    public static double CLOSE_SHOOTER_RPM = 2500;
    public static double CLOSE_CR_RPM = 2500;
    public static double FAR_SHOOTER_RPM = 3500;
    public static double FAR_CR_RPM = 3500;
    public static double RPM_TOLERANCE = 50;

    private DcMotorEx shooterMotor;
    private DcMotorEx counterRoller;
    private Servo blocker;
    private Telemetry telemetry;

    private double targetRPM = 0;
    private double crTargetRPM = 0;

    private double integral = 0;
    private double lastError = 0;

    private double crIntegral = 0;
    private double crLastError = 0;

    private long lastTime = 0;

    private ShooterState state = ShooterState.IDLE;

    public enum ShooterState {
        IDLE,
        SPINNING_UP_CLOSE,
        SPINNING_UP_FAR,
        READY_CLOSE,
        READY_FAR,
        STOPPING
    }

    public Shooter2(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        counterRoller = hardwareMap.get(DcMotorEx.class, "counterRoller");
        blocker = hardwareMap.get(Servo.class, "blocker");
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        blockerClose();
        telemetry.addData("Shooter", "Initialized");
        telemetry.addData("Counter Roller", "Initialized");
        telemetry.addData("Blocker", "Initialized");
        telemetry.update();
        lastTime = System.nanoTime();
    }

    public void blockerOpen() {
        blocker.setPosition(90);
    }

    public void blockerClose() {
        blocker.setPosition(0);
    }

    public void setState(ShooterState newState) {
        this.state = newState;
    }

    public ShooterState getState() {
        return state;
    }

    public void spinUpClose() {
        setState(ShooterState.SPINNING_UP_CLOSE);
    }

    public void spinUpFar() {
        setState(ShooterState.SPINNING_UP_FAR);
    }

    public void stop() {
        setState(ShooterState.STOPPING);
    }

    public double getShooterRPM() {
        double velocity = shooterMotor.getVelocity();
        return (velocity * 60.0) / TICKS_PER_REV;
    }

    public double getCounterRollerRPM() {
        double velocity = counterRoller.getVelocity();
        return (velocity * 60.0) / CR_TICKS_PER_REV;
    }

    public boolean atTargetSpeed() {
        double shooterRPM = getShooterRPM();
        double crRPM = getCounterRollerRPM();
        return Math.abs(targetRPM - shooterRPM) < RPM_TOLERANCE &&
                Math.abs(crTargetRPM - crRPM) < RPM_TOLERANCE;
    }

    @Override
    public void update() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        switch (state) {
            case IDLE:
                targetRPM = 0;
                crTargetRPM = 0;
                blockerClose();
                resetPID();
                break;

            case SPINNING_UP_CLOSE:
                targetRPM = CLOSE_SHOOTER_RPM;
                crTargetRPM = CLOSE_CR_RPM;
                blockerClose();

                if (atTargetSpeed()) {
                    setState(ShooterState.READY_CLOSE);
                }
                break;

            case SPINNING_UP_FAR:
                targetRPM = FAR_SHOOTER_RPM;
                crTargetRPM = FAR_CR_RPM;
                blockerClose();

                if (atTargetSpeed()) {
                    setState(ShooterState.READY_FAR);
                }
                break;

            case READY_CLOSE:
                targetRPM = CLOSE_SHOOTER_RPM;
                crTargetRPM = CLOSE_CR_RPM;
                blockerOpen();
                break;

            case READY_FAR:
                targetRPM = FAR_SHOOTER_RPM;
                crTargetRPM = FAR_CR_RPM;
                blockerOpen();
                break;

            case STOPPING:
                targetRPM = 0;
                crTargetRPM = 0;
                blockerClose();

                if (getShooterRPM() < 100 && getCounterRollerRPM() < 100) {
                    setState(ShooterState.IDLE);
                }
                break;
        }

        updateShooterMotor(dt);
        updateCounterRollerMotor(dt);

        telemetry.addData("State", state);
        telemetry.addData("Shooter Current RPM", getShooterRPM());
        telemetry.addData("Shooter Target RPM", targetRPM);
        telemetry.addData("Counter Roller Current RPM", getCounterRollerRPM());
        telemetry.addData("Counter Roller Target RPM", crTargetRPM);
        telemetry.addData("At Speed", atTargetSpeed());
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        crIntegral = 0;
        crLastError = 0;
    }

    private void updateShooterMotor(double dt) {
        double currentRPM = getShooterRPM();
        double error = targetRPM - currentRPM;

        integral += error * dt;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;

        double power = (KP * error) + (KI * integral) + (KD * derivative);
        power = Range.clip(power, 0, 1);

        shooterMotor.setPower(power);
        lastError = error;
    }

    private void updateCounterRollerMotor(double dt) {
        double currentRPM = getCounterRollerRPM();
        double error = crTargetRPM - currentRPM;

        crIntegral += error * dt;
        double derivative = (dt > 0) ? (error - crLastError) / dt : 0;

        double power = (CR_KP * error) + (CR_KI * crIntegral) + (CR_KD * derivative);
        power = Range.clip(power, 0, 1);

        counterRoller.setPower(power);
        crLastError = error;
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.a) {
            spinUpClose();
        }

        if (gp1.y) {
            spinUpFar();
        }

        if (gp1.b) {
            stop();
        }
    }
}