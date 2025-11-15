package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Kicker implements Subsystem{

    public Servo kicker;

    boolean kickerState = false;

    Telemetry telemetry;

    double kickerPos;

    public Kicker(HardwareMap hardwareMap, Telemetry telemetry) {
        kicker = hardwareMap.get(Servo.class, "kicker");
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        telemetry.addData("kicker","Initialized");
        telemetry.update();

    }

    @Override
    public void update() {
        kicker.setPosition(kickerPos);
        telemetry.addData("Kicker state", kickerState ? "Down" : "Up");
        telemetry.update();

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp2.dpadUpWasPressed()){
            kicker.setPosition(0);
            kickerState = false;
        }
        if (gp2.dpadDownWasPressed()){
            kicker.setPosition(0.3);
            kickerState = true;

        }

    }
}
