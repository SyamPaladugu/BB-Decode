package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake implements Subsystem{
    public DcMotorEx outtake;
    Telemetry telemetry;
    public double outtakePower;

    boolean outakeToggle;

    public Outtake (HardwareMap hardwareMap, Telemetry telemetry){
        outtake  = hardwareMap.get(DcMotorEx.class, "outtake");
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        telemetry.addData("Outtake", "initialized");
        telemetry.update();
    }
    public double getRPM(){

        return outtake.getVelocity();
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
        if (outtake.getVelocity() >= 2350){
            gp1.rumble(1000);
        }

    }
}
