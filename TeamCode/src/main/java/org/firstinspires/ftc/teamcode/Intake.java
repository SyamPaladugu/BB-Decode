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
    DcMotor pivot;
    DcMotor belts;
    Telemetry telemetry;
    ElapsedTime time;

    public Intake(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        belts = map.get(DcMotor.class,"belts");
        pivot = map.get(DcMotor.class,"pivot");
        double ticks = 5801.1;
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }


}
