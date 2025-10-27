package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface Subsystem {
    public void init();
    public void update();
    public void updateCtrls(Gamepad gp1, Gamepad gp2);
}