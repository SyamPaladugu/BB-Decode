package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake implements Subsystem{
    public DcMotor outtake;
    Telemetry telemetry;
    public double outtakePower;

    boolean outakeToggle;

    public Outtake (HardwareMap hardwareMap, Telemetry telemetry){
        hardwareMap.get(DcMotor.class, "outtake");
    }
    @Override
    public void init() {
        telemetry.addData("Outtake", "initialized");
        telemetry.update();
    }

    @Override
    public void update() {
        outtake.setPower(outtakePower);

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.rightBumperWasPressed()){
            outakeToggle = !outakeToggle;
            if (outakeToggle){
                outtakePower = 1;
            }else {
                outtakePower = 0;
            }
        }

    }
}
