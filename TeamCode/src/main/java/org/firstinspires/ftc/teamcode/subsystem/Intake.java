package org.firstinspires.ftc.teamcode.subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Intake  implements Subsystem {
    DcMotor rollers;

    Telemetry telemetry;
    ElapsedTime time;

    public Intake(HardwareMap map){
        this.telemetry = telemetry;
        rollers = map.get(DcMotor.class,"intake");
    }

    public void getBall(double power){
        rollers.setPower(-power);
//        telemetry.addData("Balls picked up",rollers.getPower());
//        telemetry.update();
    }

    public void removeBall(double power){
        rollers.setPower(power);
//        telemetry.addData("Ball removed",rollers.getPower());
//        telemetry.update();
    }


    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {

    }
}
