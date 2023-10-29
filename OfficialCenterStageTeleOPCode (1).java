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

@TeleOp(name="Centerstage TeleOP", group="Linear Opmode")

public class OfficialCenterStageTeleOPCode extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        
        //initialize drivetrain
        Hardware robot = new Hardware();
        robot.initDrive(this);
        robot.encoderState("reset");
        robot.encoderState("run");
        
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            double maxPower = 0.5;
            
            /* GAMEPAD 1 */
            // movement
            double up = gamepad1.left_stick_y;
            double right = -gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;
            robot.robotCentric(up, right, turn, maxPower);
            
            // linear slides up
            if(gamepad1.y){
                robot.lift.setPower(0.5); // TEST POSITIONS
            }
            
            // linear slides down
            if(gamepad1.a){
                robot.lift.setPower(-0.5);// TEST POSITIONS
            }
            
            // scoring arm + claw
            
            
            /* GAMEPAD 2 */
            // intake servo
            if(gamepad2.dpad_up){
                //robot.intakeServo.setPosition(0); // TEST POSITIONS
                robot.intakeMotor.setPower(0);
            }
            if(gamepad2.dpad_down){
                //robot.intakeServo.setPosition(0); // TEST POSITIONS
                robot.intakeMotor.setPower(0);
            }
            
        }
    }
    
}
