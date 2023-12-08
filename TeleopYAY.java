package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOP", group="Linear Opmode")

public class TeleopYAY extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
   
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
       
       
        //initialize everything
        Hardware robot = new Hardware();
        robot.initDrive(this);
        robot.encoderState("reset");
        robot.encoderState("run");
       
        // toggles
        boolean intakeToggle = false;
        boolean clawToggle = false;
       
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            double maxPower = 0.5;
           
            /* GAMEPAD 1 */
            // movement
            double up = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;
            robot.robotCentric(up, right, turn, maxPower);
           
           
       
            /*
            // intake servo and motor (doesnt work)
            if(gamepad1.b && intakeToggle == false){
                intakeToggle = true;
                if(robot.intakeServo.getPosition() == 0.92){
                    robot.intakeServo.setPosition(0.12);
                   
                } else {
                    robot.intakeServo.setPosition(0.92);

                }
            } else if (!gamepad1.b){
                intakeToggle = false;
            }
         
            if(gamepad1.b && intakeToggle == false){
                intakeToggle = true;
                if(robot.intakeMotor.getPower() == -0.5){
                    robot.intakeMotor.setPower(0);
                   
                } else {
                    robot.intakeMotor.setPower(-0.5);

                }
            } else if (!gamepad1.b){
                intakeToggle = false;
            }
           
            */
            //intake motor
            if (gamepad1.x) {
                robot.intakeMotor.setPower(-0.6);
            }
            if (gamepad1.b) {
                robot.intakeMotor.setPower(0);
            }
           
            // hanging + winch
            if(gamepad1.right_trigger>0.5){
                robot.hanging.setPower(0.8);
               robot.winch.setPower(1);
            } else if(gamepad1.left_trigger>0.5){
                robot.hanging.setPower(-0.8);
            } else {
            robot.hanging.setPower(0);
            robot.winch.setPower(0);
           
            }
           
           
            if (gamepad1.y){
                robot.winch.setPower(1);
            }
            else robot.winch.setPower(0);
           
            if (gamepad1.a){
                robot.winch.setPower(-1);
            }
            else robot.winch.setPower(0);
           
            // drone launch
            if(gamepad1.right_bumper){
                robot.drone.setPosition(0);
            }
           
           
           
           
            /* GAMEPAD 2 */
            // arm servos but better...
            if(gamepad2.a){
                robot.claw.setPosition(0.5);
               
                robot.clawWrist.setPosition(1);

                robot.scoringArmLeft.setPosition(0.1); //0.1
                robot.scoringArmRight.setPosition(0.85); //.85
               
                robot.lift.setTargetPosition(10);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (robot.lift.getCurrentPosition() == 2000) {
                    robot.lift.setPower(0.7);
                }
                else robot.lift.setPower(0.2);
               
            } else if(gamepad2.y){
                robot.lift.setTargetPosition(900);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
               
                robot.scoringArmLeft.setPosition(0.98);
                robot.scoringArmRight.setPosition(0.03);
               
                robot.clawWrist.setPosition(0.7);
               
            }
           
            // wrist
            /*
            if(gamepad2.b){
                robot.clawWrist.setPosition(0);
               
            } else if(gamepad2.x){
                robot.clawWrist.setPosition(1);
               
            }
            */
           
            // claw
            if(gamepad2.right_bumper && clawToggle == false){
                clawToggle = true;
                if(robot.claw.getPosition() == 0.3){
                    robot.claw.setPosition(0.5);
                } else {
                    robot.claw.setPosition(0.3);
                }
            } else if (!gamepad2.right_bumper){
                clawToggle = false;
            }
           
            // linear slides up
            if(gamepad2.dpad_up){
                robot.lift.setTargetPosition(2000);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.3);
            }
           
           
        }
    }
