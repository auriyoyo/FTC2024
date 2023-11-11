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
            
            // intake

            // drone

            // hanging
            
            
            /* GAMEPAD 2 */
            // scoring automation (arms, wrist, claw, slides)

            // claw
            
        }
    }
    
}