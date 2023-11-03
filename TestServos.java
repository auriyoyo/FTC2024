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
    Servo scoringArmLeft;
    Servo scoringArmRight;
    Servo clawWrist;
    Servo claw;
    
    double scoringArmLeftPos;
    double scoringArmRightPos;
    double clawWristPos;
    double clawPos;
    
    public void runOpMode(){
        
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        scoringArmLeft = hardwareMap.get(Servo.class, "scoringArmLeft");
        scoringArmRight = hardwareMap.get(Servo.class, "scoringArmRight");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        claw = hardwareMap.get(Servo.class, "claw");
        
        scoringArmLeftPos = 0.46;
        scoringArmLeft.setPosition(scoringArmLeftPos);
        scoringArmRightPos = 0.50;
        scoringArmRight.setPosition(scoringArmRightPos);
        
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            
            // arm servos
            if(gamepad1.dpad_up){
                
                scoringArmLeftPos = 0.98; 
                scoringArmLeft.setPosition(scoringArmLeftPos);
                
                scoringArmRightPos = 0; 
                scoringArmRight.setPosition(scoringArmRightPos);
                
            } else if(gamepad1.dpad_down){
                
                scoringArmLeftPos = 0.03;
                scoringArmLeft.setPosition(scoringArmLeftPos);
                
                scoringArmRightPos = 0.95;
                scoringArmRight.setPosition(scoringArmRightPos); 
                
            }
            telemetry.addData("left arm position", scoringArmLeft.getPosition());
            telemetry.addData("right arm position", scoringArmRight.getPosition());
            telemetry.update();
            
            // wrist
            if(gamepad1.b){
                clawWristPos = 0; 
                clawWrist.setPosition(clawWristPos);
            } else if(gamepad1.x){
                clawWristPos = 1; 
                clawWrist.setPosition(clawWristPos);
            }
            telemetry.addData("claw wrist", clawWrist.getPosition());
            telemetry.update();
            
            // claw
            if(gamepad1.a){
                clawPos = 0; 
                claw.setPosition(clawPos);
            } else if(gamepad1.y){
                clawPos = 1; 
                claw.setPosition(clawPos);
            }
            telemetry.addData("claw", claw.getPosition());
            telemetry.update();
            
        }
    }
}
