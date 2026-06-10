package org.firstinspires.ftc.teamcode.teleop.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadMapping {
    // GAMEPADS
    public final Gamepad gamepad1;
    public final Gamepad gamepad2;

    // DRIVETRAIN
    // --------------
    public static double drive = 0.0;
    public static double strafe = 0.0;
    public static double turn = 0.0;

    // INTAKE
    public Toggle intakeTransfer;
    public Toggle intakeOnly;

    // SHOOTER - GP2
    public Toggle shootBack;
    public Toggle shootFront;

    // MISC
    public Toggle transfer; // LM1
    public Toggle outtake; // LM1
    public Toggle switchMode; // switch btwn PID shoot and hardcoded
    public Toggle park;

    public GamepadMapping(Gamepad gamepad1, Gamepad gamepad2) {
        // GAMEPADS
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // INTAKE
        intakeTransfer = new Toggle(false);
        intakeOnly = new Toggle(false);

        // SHOOTER
        shootBack = new Toggle(false);
        shootFront = new Toggle(false);

        // MISC
        transfer = new Toggle(false);
        outtake = new Toggle(false);
        switchMode = new Toggle(false);
        park = new Toggle(false);
    }

    public void joystickUpdate() {
        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
    }

    public void update() {
        joystickUpdate();

        //Auto align - Right Trigger
        //Mecanum override - Left Trigger
        // INTAKE
        intakeTransfer.update(gamepad1.left_bumper);
        intakeOnly.update(gamepad1.left_trigger > 0);

        // TRANSFER
        transfer.update(gamepad1.right_bumper);

        // OUTTAKE
        outtake.update(gamepad1.b);

        // MISC
        switchMode.update(gamepad1.dpad_down);

        // SHOOTER - FOR HARDCODED ONLY
        shootBack.update(gamepad1.right_trigger >= 0.3);
        shootFront.update(gamepad1.right_bumper);


    }

    public void resetMultipleControls(Toggle... toggles) {
        for (Toggle toggle : toggles) {
            toggle.set(false);
        }
    }
}

