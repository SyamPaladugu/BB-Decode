package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Shooter implements Subsystem {
    // Shooter PID constants
    public static double KP = 0.01;
    public static double KI = 0.0;
    public static double KD = 0.0;

    // Counter Roller PID constants
    public static double CR_KP = 0.01;
    public static double CR_KI = 0.0;
    public static double CR_KD = 0.0;

    public static double TICKS_PER_REV = 28.0; // Adjust for your shooter motor
    public static double CR_TICKS_PER_REV = 28.0; // Adjust for your counter roller motor
    public static double TARGET_RPM = 3000;
    public static double CR_TARGET_RPM = 3000;
    public static double RPM_TOLERANCE = 50;

    private DcMotorEx shooterMotor;
    private DcMotorEx counterRoller;
    private Telemetry telemetry;

    private double targetRPM = 0;
    private double crTargetRPM = 0;

    // Shooter PID variables
    private double integral = 0;
    private double lastError = 0;

    // Counter Roller PID variables
    private double crIntegral = 0;
    private double crLastError = 0;

    private long lastTime = 0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        counterRoller = hardwareMap.get(DcMotorEx.class, "counterRoller");
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Shooter", "Initialized");
        telemetry.addData("Counter Roller", "Initialized");
        telemetry.update();
        lastTime = System.nanoTime();
    }

    /**
     * Set the target RPM for the shooter
     */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    /**
     * Set the target RPM for the counter roller
     */
    public void setCounterRollerTargetRPM(double rpm) {
        this.crTargetRPM = rpm;
    }

    /**
     * Set the target RPM for both shooter and counter roller
     */
    public void setTargetRPM(double shooterRPM, double counterRollerRPM) {
        this.targetRPM = shooterRPM;
        this.crTargetRPM = counterRollerRPM;
    }

    
    public void stop() {
        targetRPM = 0;
        crTargetRPM = 0;
        integral = 0;
        lastError = 0;
        crIntegral = 0;
        crLastError = 0;
    }


    public double getShooterRPM() {
        double velocity = shooterMotor.getVelocity(); // ticks per second
        return (velocity * 60.0) / TICKS_PER_REV;
    }


    public double getCounterRollerRPM() {
        double velocity = counterRoller.getVelocity(); // ticks per second
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
        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds
        lastTime = currentTime;

        // Update shooter motor
        updateShooterMotor(dt);

        // Update counter roller motor
        updateCounterRollerMotor(dt);

        // Telemetry
        telemetry.addData("Shooter Current RPM", getShooterRPM());
        telemetry.addData("Shooter Target RPM", targetRPM);
        telemetry.addData("Counter Roller Current RPM", getCounterRollerRPM());
        telemetry.addData("Counter Roller Target RPM", crTargetRPM);
        telemetry.addData("At Speed", atTargetSpeed());
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
            setTargetRPM(TARGET_RPM, CR_TARGET_RPM);
        } else if (gp1.b) {
            stop();
        }
    }
}