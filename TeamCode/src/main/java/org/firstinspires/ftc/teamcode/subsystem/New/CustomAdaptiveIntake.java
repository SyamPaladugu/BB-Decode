package org.firstinspires.ftc.teamcode.subsystem.New;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

public class CustomAdaptiveIntake implements Subsystem {
    public DcMotor intake;
    Telemetry telemetry;
    public double intakePower;

    Servo intakePiv;
    public CustomAdaptiveIntake (HardwareMap hardwareMap, Telemetry telemetry){
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakePiv = hardwareMap.get(Servo.class, "intakePiv");
        this.telemetry = telemetry;
    }


    @Override
    public void init() {
        telemetry.addData("Intake","Initialized");
        telemetry.addData("PivServo", "Initialized");
        telemetry.update();

    }

    public void intakeIn(double power) {

        intakePower = power;
    }
    public void autoIntake(){
        intake.setPower(1);
    }

    public void autoIntakeOff(){
        intake.setPower(0);
    }

    public void pivSendBalls(){
        intakePiv.setPosition(0); // changePos based on irl
    }
    public void pivIntake(){
        intakePiv.setPosition(2); // changePos based on irl
    }

    @Override
    public void update() {
        intake.setPower(intakePower);
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        intakePower = gp1.right_trigger;
        if (gp2.aWasPressed()){
            pivSendBalls();
        }
        if (gp2.bWasPressed()){
            pivIntake();
        }

    }
}