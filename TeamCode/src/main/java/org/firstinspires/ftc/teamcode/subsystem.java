package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface subsystem {
    public void init();
    public void update();
    public void updateCtrls(Gamepad gp1, Gamepad gp2);
}