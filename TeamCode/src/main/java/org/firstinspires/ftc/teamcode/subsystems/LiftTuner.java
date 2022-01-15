package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class LiftTuner extends LinearOpMode {

    public static double SPOOL_SIZE_IN = 20.0/25.4;
    public static double MOTOR_RATIO = 5.2;
    public static double TICKS_PER_REV = MOTOR_RATIO * 28.0;
    public static double GEAR_RATIO = 20.0/24.0;

    public static double lpower = 0;
    public static double rpower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left = hardwareMap.get(DcMotor.class, "lift1");
        DcMotor right = hardwareMap.get(DcMotor.class, "lift2");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            left.setPower(lpower);
            right.setPower(rpower);

            telemetry.addData("Left pos", encoderTicksToInches(left.getCurrentPosition()));
            telemetry.addData("Left pos", encoderTicksToInches(right.getCurrentPosition()));
            telemetry.update();
        }
    }

    public static double encoderTicksToInches(double ticks) {
        return SPOOL_SIZE_IN * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
