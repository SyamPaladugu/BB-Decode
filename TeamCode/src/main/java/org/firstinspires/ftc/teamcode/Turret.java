package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    private DcMotor motor;
    private Servo servo;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    TrackAprilTag tag;
    private double currentPos;

    public Turret(DcMotor motor,Servo servo){
       this.motor = motor;
       this.servo = servo;
    }
    //TODO: rename ts
     public void findTag(){
        boolean tracked = true;
        while (tag.stuff()[2]==0) {
            currentPos +=tag.stuff()[2];
            servo.setPosition(currentPos + tag.stuff()[2]);
        }
     }

}
