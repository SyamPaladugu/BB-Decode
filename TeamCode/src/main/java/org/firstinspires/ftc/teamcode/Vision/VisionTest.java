package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@TeleOp(name="VisionTest", group = " ")
public class VisionTest extends OpMode {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    AprilTag aprilTag = new AprilTag();
    @Override
    public void init(){
        aprilTag.init(hardwareMap,telemetry);


    }

    @Override
    public void loop() {
        AprilTagDetection id20 = aprilTag.getTagByID(20);
        update();
        aprilTag.displayTelemtry(id20);
        telemetry.update();w


    }

    public void update(){
        aprilTag.updateCtrls(gamepad1,gamepad2);
    }
}