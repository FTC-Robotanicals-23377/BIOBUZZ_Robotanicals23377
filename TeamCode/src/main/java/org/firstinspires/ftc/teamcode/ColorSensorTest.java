//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name = "Color Sensor Test")
//public class ColorSensorTest extends OpMode {
//
//    NormalizedColorSensor colorsensor;
//    Servo launcherServo;
//
//    @Override
//    public void init() {
//        // Initialize hardware directly using this OpMode's hardwareMap
//        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
//        launcherServo = hardwareMap.get(Servo.class, "launcher_servo");
//        launcherServo.setPosition(0.0);
//
//        telemetry.addData("Status", "Initialized Color Sensor Test");
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        NormalizedRGBA colors = colorsensor.getNormalizedColors();
//        float alpha = colors.alpha;
//
//        if (alpha > 0.02) {
//            float normRed = colors.red / alpha;
//            float normGreen = colors.green / alpha;
//            float normBlue = colors.blue / alpha;
//
//            double currentPos = launcherServo.getPosition();
//
//            // Yellow = Default position (0.0)
//            if (normRed > 0.35 && normGreen > 0.35 && normBlue < 0.25) {
//                launcherServo.setPosition(0.0);
//                telemetry.addData("Detected Color", "Yellow (Position 0.0)");
//            }
//            // Red or Blue = Go back by 0.2
//            else if ((normRed > normGreen && normRed > normBlue) || (normBlue > normRed && normBlue > normGreen)) {
//                double newPos = Math.max(0.0, currentPos - 0.2);
//                launcherServo.setPosition(newPos);
//                telemetry.addData("Detected Color", "Red/Blue -> Moving back");
//            }
//
//            telemetry.addData("Norm Red", normRed);
//            telemetry.addData("Norm Green", normGreen);
//            telemetry.addData("Norm Blue", normBlue);
//        } else {
//            telemetry.addData("Status", "No object detected");
//            launcherServo.setPosition(0.0); // Default position
//        }
//
//        telemetry.update();
//    }
//}