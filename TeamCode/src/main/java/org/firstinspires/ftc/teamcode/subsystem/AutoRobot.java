package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoRobot {
    public Intake intake;
    public Outtake outtake;

    public Timer timer;

    public boolean bool = false;

    public AutoRobot(HardwareMap map, Telemetry telemetry){
        intake = new Intake(map, telemetry);
        outtake = new Outtake(map, telemetry);

    }
    public void update(){
        intake.update();
        outtake.update();
    }


    public double timer() {return timer.getElapsedTimeSeconds();}

    public class WaitSeconds implements Action {

        private double delay;

        public WaitSeconds(double delay) {
            this.delay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!bool) {
                timer.resetTimer();
                bool = true;
            }

            if (timer() > delay) {
                bool = false;
                return false;
            }

            return true;

        }
    }

    public Action waitSeconds(double delay) {return new WaitSeconds(delay);}

    public class Init implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
}
