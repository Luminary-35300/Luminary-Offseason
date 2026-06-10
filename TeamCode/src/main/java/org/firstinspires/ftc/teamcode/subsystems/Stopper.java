package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Stopper {
    public final ServoImplEx stopper;

    public Stopper(HardwareMap hardwareMap) {
        stopper = hardwareMap.get(ServoImplEx.class, "stopper");
    }

    public Stopper(ServoImplEx stopper) {
        this.stopper = stopper;
    }
// ------------------------------------------------------------------

    public void release() {
        stopper.setPosition(0.9);
    }
    public void stop() {
        stopper.setPosition(0.7);
    }
}

