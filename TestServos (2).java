package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestServos", group="LinearOpmode")

public class TestServos extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Servo intakeServo;
    Servo scoringArm1;
    Servo scoringArm2;
    Servo clawWrist;
    Servo claw;
    
    public void runOpMode(){
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            
            // intakeServo = hardwareMap.get(Servo.class, "intakeServo");
            scoringArm1 = hardwareMap.get(Servo.class, "somethingsimplefornow");
            scoringArm2 = hardwareMap.get(Servo.class, "scoringArm2");
            // clawWrist = hardwareMap.get(Servo.class, "clawWrist");
            // claw = hardwareMap.get(Servo.class, "claw");
            
            // arm servos
            if(gamepad1.dpad_up){
                scoringArm1.setPosition(0);
                telemetry.addData("arm 1 position", scoringArm1.getPosition());
                telemetry.update();
                
            } else if(gamepad1.dpad_down){
                scoringArm1.setPosition(1);
                telemetry.addData("arm 1 position", scoringArm1.getPosition());
                telemetry.update();
            }
            
            
            if(gamepad1.y){
                scoringArm2.setPosition(0); // spin down
                telemetry.addData("arm 2 position", scoringArm2.getPosition());
                telemetry.update();
            } else if(gamepad1.a){
                scoringArm2.setPosition(1); // spin up
                telemetry.addData("arm 2 position", scoringArm2.getPosition());
                telemetry.update();
            }
            
            /*
            // intake servo
            if(gamepad1.x){
                intakeServo.setPosition(scoringArm1.getPosition()+0.1);
            } else if(gamepad1.b){
                intakeServo.setPosition(scoringArm1.getPosition()+0.1);
            }
            */
        }
    }
}
