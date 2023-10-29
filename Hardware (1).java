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

public class Hardware {
    
    private LinearOpMode myOpMode;
    private ElapsedTime runtime = new ElapsedTime();
    
    // motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    public DcMotor lift;
    public DcMotor intakeMotor;
    
    // servos
    public Servo intakeServo;
    public Servo scoringArm1;
    public Servo scoringArm2;
    public Servo clawWrist;
    public Servo claw;
    
    // drive by counts
    static final double COUNTS_PER_MOTOR_REV    = 751.1 ;    
    static final double COUNTS_PER_DEGREE       = COUNTS_PER_MOTOR_REV / 360;
    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;      
    static final double WHEEL_DIAMETER_INCHES   = 3.75 ;     
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    
    // imu for robot centric
    private BNO055IMU imu;
    
    // constructor
    public Hardware(){
    }
    
    public void initDrive(LinearOpMode opMode){
        myOpMode = opMode;
        
        // initialize motors
        frontLeft  = myOpMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeft  = myOpMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "backRightDrive");
        lift = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        intakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        
        // set motor direction
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        
        // try completely break 
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // set power 0
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        lift.setPower(0);
        intakeMotor.setPower(0);
        
        // imu initialize from hardware map
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        
        // initialize servos
        // *************** NEED TEST THIS
        intakeServo = myOpMode.hardwareMap.get(Servo.class, "intakeServo");
        //intakeServo.setPosition(0.5);
        scoringArm1 = myOpMode.hardwareMap.get(Servo.class, "scoringArm1");
        //scoringArm1.setPosition(0.5);
        scoringArm2 = myOpMode.hardwareMap.get(Servo.class, "scoringArm2");
        //scoringArm2.setPosition(0.5);
        clawWrist = myOpMode.hardwareMap.get(Servo.class, "clawWrist");
        //clawWrist.setPosition(0.5);
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        //claw.setPosition(0.5);
    }
    
    public void encoderState(String state){
        if(state.equals("reset")){
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if(state.equals("run")){
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if(state.equals("position")){
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        } else if(state.equals("off")){
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    
    public void robotCentric(double up, double right, double turn, double maxPower){
        double frontLeftPower    = Range.clip(up +right -turn, -1, 1);
        double frontRightPower   = Range.clip(up -right +turn, -1, 1);
        double backLeftPower     = Range.clip(up -right -turn, -maxPower, maxPower);
        double backRightPower    = Range.clip(up +right +turn, -maxPower, maxPower);
           
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
    
    public void fieldCentric(double y, double x, double turn, double maxPower){
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double rotX = x*Math.cos(botHeading) - y*Math.sin(botHeading);
        double rotY = x*Math.sin(botHeading) + y*Math.cos(botHeading);
        
        double frontleftPower    = Range.clip(rotY +rotX -turn, -maxPower, maxPower);
        double frontrightPower   = Range.clip(rotY -rotX +turn, -0.3, 0.3);
        double backleftPower     = Range.clip(rotY -rotX -turn, -maxPower, maxPower);
        double backrightPower    = Range.clip(rotY +rotX +turn, -maxPower, maxPower);
        
        frontLeft.setPower(frontleftPower);
        frontRight.setPower(frontrightPower);
        backLeft.setPower(backleftPower);
        backRight.setPower(backrightPower);
    }
    
}
