package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Intake {
    DcMotor rollers;
    DcMotor belts;
    Telemetry telemetry;
    ElapsedTime time;

    public Intake(HardwareMap map){
        this.telemetry = telemetry;
        belts = map.get(DcMotor.class,"belts");
        rollers = map.get(DcMotor.class,"rollers");
    }

    public void getBall(double power){
        rollers.setPower(power);
        belts.setPower(power-0.3);
        telemetry.addData("Balls picked up",rollers.getPower());
        telemetry.update();
    }

    public void removeBall(double power){
        rollers.setPower(-power);
        telemetry.addData("Ball removed",rollers.getPower());
        telemetry.update();
    }



}
