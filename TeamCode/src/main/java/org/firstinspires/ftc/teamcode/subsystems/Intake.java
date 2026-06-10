package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

public class Intake {
    public final DcMotorEx transfer;
    public final DcMotorEx dropdownIntake;
    public final Servo pivotServo;

    private GamepadMapping controls;

    private boolean ballIn = false;
    private int cycles = 0;
    private int lowCycles = 0;
    public Intake(HardwareMap hardwareMap, GamepadMapping controls) {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        dropdownIntake = hardwareMap.get(DcMotorEx.class, "dropdownIntake");

        pivotServo = hardwareMap.get(Servo.class, "pivotServo");

        this.controls = controls;
    }

    // Constructor for JUnit
    public Intake(DcMotorEx transfer, DcMotorEx intake2) {
        this.dropdownIntake = intake2;
        this.transfer       = transfer;
        this.pivotServo     = null;
    }

//-------------------------------------------------------------------------------

    public void setIntakePower(double power) {
        double intakePower = power;
        double transferPower = power;

        double intakeCurrent = dropdownIntake.getCurrent(CurrentUnit.AMPS);
        if (intakeCurrent > 6) {
            intakePower = intakePower * 0.9;
        }
        dropdownIntake.setPower(intakePower);

        double transferCurrent = transfer.getCurrent(CurrentUnit.AMPS);
        if (transferCurrent > 6) {
            transferPower = transferPower * 0.9;
        }
        transfer.setPower(transferPower);
    }
    public void intakeTransferOnClose() {
        double intakePower = 1;
        double transferPower = 1;

        double intakeCurrent = dropdownIntake.getCurrent(CurrentUnit.AMPS);
        if (intakeCurrent > 6) {
            intakePower = intakePower * 0.9;
        }
        dropdownIntake.setPower(intakePower);

        double transferCurrent = transfer.getCurrent(CurrentUnit.AMPS);
        if (transferCurrent > 6) {
            transferPower = transferPower * 0.9;
        }
        transfer.setPower(transferPower);
    }

    public void intakeTransferOnFar() {
        double intakePower = 1;
        double transferPower = 0.75;

        double intakeCurrent = dropdownIntake.getCurrent(CurrentUnit.AMPS);
        if (intakeCurrent > 6) {
            intakePower = intakePower * 0.65;
        }
        dropdownIntake.setPower(intakePower);

        double transferCurrent = transfer.getCurrent(CurrentUnit.AMPS);
        if (transferCurrent > 6) {
            transferPower = transferPower * 0.5;
        }
        transfer.setPower(transferPower);
    }

    public void intakeTransferOff() {
        dropdownIntake.setPower(0);
        transfer.setPower(0);
    }

    public void intakeTransferReverse() {
        double intakePower = -1;
        double transferPower = -1;

        double intakeCurrent = dropdownIntake.getCurrent(CurrentUnit.AMPS);
        if (intakeCurrent > 6) {
            intakePower = intakePower * 0.5;
        }
        dropdownIntake.setPower(intakePower);

        double transferCurrent = transfer.getCurrent(CurrentUnit.AMPS);
        if (transferCurrent > 6) {
            transferPower = transferPower * 0.5;
        }
        transfer.setPower(transferPower);
    }

    public void pivotUp() {
       pivotServo.setPosition(0.28);
    }

    public void pivotDown() {
        pivotServo.setPosition(0.88);
    }

    public void setPivotPos(double position) {
        pivotServo.setPosition(position);
    }
    public double getTransferCurrent() {
        return transfer.getCurrent(CurrentUnit.AMPS);
    }
    public void safeSetPower(double power, Gamepad gamepad) {
        if (gamepad.right_bumper) {
            ballIn = false;
            cycles = 0;
            lowCycles = 0;
        }
        if (getTransferCurrent() > 4) {
            cycles = cycles + 1;
            lowCycles = 0;
        } else {
            lowCycles = lowCycles + 1;
            if (lowCycles > 20) {
                cycles = 0;
                ballIn = false;
            }
        }
        if (cycles > 9) {
            ballIn = true;
        }
        if (!ballIn) {
            transfer.setPower(power);
        }
        else {
            transfer.setPower(0);
        }
    }


}