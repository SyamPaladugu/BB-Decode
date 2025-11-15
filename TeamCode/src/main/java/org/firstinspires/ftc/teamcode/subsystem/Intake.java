package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem{
    public DcMotor intake;
    Telemetry telemetry;
    public double intakePower;
    public Intake (HardwareMap hardwareMap, Telemetry telemetry){
        intake = hardwareMap.get(DcMotor.class, "intake");
        this.telemetry = telemetry;
    }


    @Override
    public void init() {
        telemetry.addData("Intake","Initialized");
        telemetry.update();

    }

    public void intakeIn(double power) {
        intakePower = power;
    }

    @Override
    public void update() {
        intake.setPower(intakePower);

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        intakePower = gp1.right_trigger;

    }
}