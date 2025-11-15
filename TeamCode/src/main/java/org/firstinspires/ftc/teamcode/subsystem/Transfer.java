package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer implements Subsystem{
    public CRServo transfer;
    Telemetry telemetry;

    double transferPower;
    boolean transferToggle = false;
    public Transfer(HardwareMap hardwareMap, Telemetry telemetry){
        transfer = hardwareMap.get(CRServo.class, "transfer");
        this.telemetry = telemetry;
    }


    @Override
    public void init() {
        telemetry.addData("transfer","Initialized");
        telemetry.update();

    }


    @Override
    public void update() {
        transfer.setPower(transferPower);

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp2.dpadRightWasPressed()){
            transferToggle = !transferToggle;
            if (transferToggle){
                transferPower = -1;
            }else {
                transferPower = 0;
            }
        }

    }
}