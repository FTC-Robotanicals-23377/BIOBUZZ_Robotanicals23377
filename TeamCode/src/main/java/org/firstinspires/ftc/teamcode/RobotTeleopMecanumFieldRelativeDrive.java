package org.firstinspires.ftc.teamcode;/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
import static android.graphics.Color.red;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action.*;
import com.acmerobotics.roadrunner.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Test: Field Relative Mecanum Drive")
public class
RobotTeleopMecanumFieldRelativeDrive extends OpMode {
    // This declares the four motors needed
    DcMotorEx frontLeftDrive;
    DcMotorEx frontRightDrive;
    DcMotorEx backLeftDrive;
    DcMotorEx backRightDrive;
//    DcMotorEx intakeMotor;

    private final ElapsedTime timer = new ElapsedTime();
    Servo launcherServo;

  NormalizedColorSensor colorsensor;
//    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    public PoseVelocity2d updatePoseEstimate() {
        // asks localizer to update and return robots velocity
        PoseVelocity2d vel = localizer.update();
        // adds current pose to list then drops oldest entries so 100 remain
        poseHistory.add(localizer.getPose());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }
        // logs pose and returns velocity
        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        return vel;
    }

    // canvas = drawing layer on top of field picture
    // essentially just draws a line through the points from the method above
    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

// @Override

    public enum DetectedColor{
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }
    int count = 0;
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        launcherServo = hardwareMap.get(Servo.class, "launcher_servo");
        launcherServo.setDirection(Servo.Direction.FORWARD);
        launcherServo.setPosition(0);
//        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake_motor");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    public DetectedColor getDetectedColor(Telemetry telemetry, NormalizedRGBA colors) {
        if (colors.alpha == 0) return DetectedColor.UNKNOWN;

        float normRed = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue = colors.blue / colors.alpha;

        telemetry.addData("Normalized Red", normRed);
        telemetry.addData("Normalized Green", normGreen);
        telemetry.addData("Normalized Blue", normBlue);

        return DetectedColor.UNKNOWN;
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
//        telemetry.addLine("The red value is " + colorsensor.getNormalizedColors());
//        telemetry.addLine("The green value is " + colorsensor.green());
//        telemetry.addLine("The blue value is" + colorsensor.blue());
//        telemetry.addLine("The alpha value is" + colorsensor.alpha());

        NormalizedRGBA colors = colorsensor.getNormalizedColors();
        float alpha = colors.alpha;

        if (colors.alpha > 0) {
            telemetry.addLine("The blue value is " + Double.toString(colors.blue / colors.alpha));
            telemetry.addLine("The green value is " + Double.toString(colors.green / colors.alpha));
            telemetry.addLine("The red value is " + Double.toString(colors.red / colors.alpha));
        }

        if (alpha > 0.02) { // makes sure an object is close enough to read
            float normRed = colors.red / alpha;
            float normGreen = colors.green / alpha;
            float normBlue = colors.blue / alpha;

            double currentPos = launcherServo.getPosition();

            // check for yellow (High Red and Green, Low Blue) -> Default Position
            if (normRed > 0.035 && normGreen > 0.035 && normBlue < 0.025) {
                launcherServo.setPosition(0.0); // Set to your default position
                telemetry.addData("Detected Color", "Yellow (Default Position 0.0)");
            }
            // check for Red or Blue dominant -> Go back by 0.2
            else if ((normRed > normGreen && normRed > normBlue) || (normBlue > normRed && normBlue > normGreen)) {
                double newPos = Math.max(0.0, currentPos - 0.1); // Prevents going below 0.0
                launcherServo.setPosition(newPos);
                telemetry.addData("Detected Color", "Red or Blue -> Moving back by 0.2");
            }
            // optional: Green dominant -> Go up by 0.2
            else if (normGreen > normRed && normGreen > normBlue) {
                double newPos = Math.min(1.0, currentPos + 0.1); // Prevents going above 1.0
                launcherServo.setPosition(newPos);
                telemetry.addData("Detected Color", "Green -> Moving up by 0.2");
            }

            // display normalized values for calibration
            telemetry.addData("Norm Red", normRed);
            telemetry.addData("Norm Green", normGreen);
            telemetry.addData("Norm Blue", normBlue);
        } else {
            telemetry.addData("Status", "No object detected by sensor");
        }
//         If you press the A button, then you reset the Yaw to be zero from the way
//         the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }
//         If you press the left bumper, you get a drive from the point of view of the robot
//         (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        telemetry.addLine(Integer.toString(count));
//       if (gamepad1.x) {
//           count++;
//          telemetry.addLine(Double.toString(launcherServo.getPosition()));
//           if ((launcherServo.getPosition() == 0) && timer.time() > 0.2) {
//               launcherServo.setPosition(.2);
//               timer.reset();
//            } else if (launcherServo.getPosition() == 0.2 && (timer.time() > 0.2)) {
//                launcherServo.setPosition(0);
//                timer.reset();
//            }
//        }
//        telemetry.addLine("This is the value of y: " + Float.toString(gamepad1.left_stick_y));
//        telemetry.addLine("This is the value of x: " + Float.toString(gamepad1.left_stick_x));
//        if ((-0.9 >= gamepad1.left_stick_y) && (gamepad1.left_stick_y >= -1)) {
//            frontLeftDrive.setVelocity(400);
//            frontRightDrive.setVelocity(400);
//            backRightDrive.setVelocity(400);
//            backLeftDrive.setVelocity(400);
//        }
//        if ((0.9 <= gamepad1.left_stick_y) && (gamepad1.left_stick_y <= 1)) {
//            frontLeftDrive.setVelocity(-400);
//            frontRightDrive.setVelocity(-400);
//            backLeftDrive.setVelocity(-400);
//            backRightDrive.setVelocity(-400);
//        }
//        if ((-0.9 >= gamepad1.left_stick_x) && (gamepad1.left_stick_x >= -1)) {
//            frontLeftDrive.setVelocity(400);
//            frontRightDrive.setVelocity(400);
//            backRightDrive.setVelocity(-400);
//            backLeftDrive.setVelocity(-400);
//        }
//        if ((0.9 <= gamepad1.left_stick_x) && (gamepad1.left_stick_x <= 1)) {
//            frontLeftDrive.setVelocity(-400);
//            frontRightDrive.setVelocity(-400);
//            backLeftDrive.setVelocity(400);
//            backRightDrive.setVelocity(400);
//        }
//        if ((-0.2 <= gamepad1.left_stick_y) && (gamepad1.left_stick_y <= 0.2) && (-0.2 <= gamepad1.left_stick_x) && (gamepad1.left_stick_x <= 0.2)) {
//            frontLeftDrive.setVelocity(0);
//            frontRightDrive.setVelocity(0);
//            backLeftDrive.setVelocity(0);
//            backRightDrive.setVelocity(0);
//        }
//        telemetry.addData("Status", "Initialized! Ready to drive.");
//        telemetry.update();

//         if(gamepad1.y) {
//            intakeMotor.setPower(0.7);
//        }
//        if(gamepad1.right_bumper){
//            intakeMotor.setPower(0.0);
//        }

    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 0.25;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
//public final class TurnAction implements Action {
//        private final TimeTurn turn;
//
//        private double beginTs = -1;
//
//        public TurnAction(TimeTurn turn) {
//            this.turn = turn;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket p) {
//            double t;
//            if (beginTs < 0) {
//                beginTs = Actions.now();
//                t = 0;
//            } else {
//                t = Actions.now() - beginTs;
//            }
//
//            if (t >= turn.duration) {
//                frontLeftDrive.setPower(0);
//                backLeftDrive.setPower(0);
//                backRightDrive.setPower(0);
//                frontRightDrive.setPower(0);
//
//                return false;
//            }
//
//            Pose2dDual<Time> txWorldTarget = turn.get(t);
//            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
//
//            PoseVelocity2d robotVelRobot = updatePoseEstimate();
//
//            PoseVelocity2dDual<Time> command = new HolonomicController(
//                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
//                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
//            )
//                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
//            driveCommandWriter.write(new DriveCommandMessage(command));
//
//            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
//            double voltage = voltageSensor.getVoltage();
//            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
//                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
//            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
//            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
//            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
//            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
//            mecanumCommandWriter.write(new MecanumCommandMessage(
//                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
//            ));
//
//            frontLeftDrive.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
//            backLeftDrive.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
//            backRightDrive.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
//            frontRightDrive.setPower(feedforward.compute(wheelVels.rightFront) / voltage);
//
//            Canvas c = p.fieldOverlay();
//            drawPoseHistory(c);
//
//            c.setStroke("#4CAF50");
//            Drawing.drawRobot(c, txWorldTarget.value());
//
//            c.setStroke("#3F51B5");
//            Drawing.drawRobot(c, localizer.getPose());
//
//            c.setStroke("#7C4DFFFF");
//            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
//
//            return true;
//        }
//
//        public void preview(Canvas c) {
//            c.setStroke("#7C4DFF7A");
//            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
//        }
//    }

//    }
