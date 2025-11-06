package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoRobot {
    public Intake intake;
    public Outtake outtake;

    public ElapsedTime timer;

    public AutoRobot(HardwareMap map, Telemetry telemetry){
        intake = new Intake(map, telemetry);
        outtake = new Outtake(map, telemetry);

    }
    public void update(){
        intake.update();
        outtake.update();
    }
}
