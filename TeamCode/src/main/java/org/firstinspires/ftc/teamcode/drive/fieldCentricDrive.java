package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.*;

//import com.acmerobotics.roadrunner.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Action.*;
import com.acmerobotics.roadrunner.*;

import org.firstinspires.ftc.teamcode.RobotTeleopMecanumFieldRelativeDrive;

import java.lang.Math;

@Autonomous(name = "SampleAuto")
public class SampleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(270));

        RobotTeleopMecanumFieldRelativeDrive drive = new RobotTeleopMecanumFieldRelativeDrive(hardwareMap, beginPose);

        waitForStart();


        Action path = drive.actionBuilder(beginPose)
        .splineToSplineHeading(new Pose2d(40,0, Math.toRadians(180)), Math.toRadians(270)
        .splineToConstantHeading(new Vector2d(40,-48), Math.toRadians(0))
        .lineToXLinearHeaeding(0,Math.toRadians(90))
                .build();


        Actions.runBlocking(new SequentialAction(path));

    }

}