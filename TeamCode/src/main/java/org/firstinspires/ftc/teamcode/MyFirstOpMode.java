package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Two Motor Drive", group = "Tutorial")
public class MyFirstOpMode extends LinearOpMode {

    // 1. Declare both motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        // 2. Map them to the names you will use in your Robot Configuration
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        telemetry.addData("Status", "Initialized! Ready to drive.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get stick position (-1.0 to 1.0). Invert Y because pushing up is negative in Android.
            double drivePower = -gamepad1.left_stick_y;

            // Send that power directly to both motors simultaneously
            leftMotor.setPower(drivePower);
            rightMotor.setPower(drivePower);

            // Send live data back to your phone/screen display
            telemetry.addData("Stick Position", drivePower);
            telemetry.addData("Left Motor Power", leftMotor.getPower());
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.update();
        }
    }
}

