package org.firstinspires.ftc.teamcode.teleop.opModes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@TeleOp(name = "Drivetrain Test", group = "test")
public class DrivetrainTest extends LinearOpMode {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private Intake intake;
    private GamepadMapping controls;


    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        controls = new GamepadMapping(gamepad1, gamepad2);
        intake = new Intake(hardwareMap, controls);



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double vertical   = gamepad1.left_stick_y;
            double strafe =  -gamepad1.left_stick_x;
            double turn     =  -gamepad1.right_stick_x;

            double flPower = vertical + strafe + turn;
            double frPower = vertical - strafe - turn;
            double blPower = vertical - strafe + turn;
            double brPower = vertical + strafe - turn;

            double maxPower = Math.max(1.0,
                    Math.max(
                            Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))
                    )
            );

            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;

            leftFront.setPower(flPower);
            rightFront.setPower(frPower);
            leftBack.setPower(blPower);
            rightBack.setPower(brPower);

            telemetry.addData("Front Left  Power", "%.2f", flPower);
            telemetry.addData("Front Right Power", "%.2f", frPower);
            telemetry.addData("Back  Left  Power", "%.2f", blPower);
            telemetry.addData("Back  Right Power", "%.2f", brPower);
            telemetry.addData("real power", rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Slow Mode", gamepad1.right_bumper ? "ON" : "OFF");
            telemetry.update();
        }
    }
}