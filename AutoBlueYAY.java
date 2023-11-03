package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class AutoBlueYAY extends LinearOpMode {
    @Override
    public void runOpMode(){
        ElapsedTime runtime = new ElapsedTime();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // initialize everyhting
        Hardware robot = new Hardware();
        OpenCvWebcam webcam;
        
        // initialize webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            Vision pipeline = new Vision();
            webcam.setPipeline(pipeline);
            
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
                public void onOpened(){
                    webcam.startStreaming(1280, 960,OpenCvCameraRotation.UPRIGHT);
                }
                public void onError(int errorCode){
                }
            });
            
        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();
          
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        
        String snapshotAnalysis = pipeline.getAnalysis();
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();
        
        /*    
        switch (snapshotAnalysis)
            {
                case "left":        
                {
                
                }
                
                case "center":        
                {
                
                }
                
                case "right":        
                {
                
                }
            }
            
        */
    }
    public class Vision extends OpenCvPipeline {

    String position; 

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // creates HSV image
        Imgproc.GaussianBlur(mat, mat, new Size(15, 15), 0);
        
        /*
        RED
        Scalar lowHSV = new Scalar(0, 155, 175);
        Scalar highHSV = new Scalar(0, 195, 195);
        
        BLUE
        Scalar lowHSV = new Scalar(105, 70, 70);
        Scalar highHSV = new Scalar(125, 110, 110);
        */
        
        Scalar lowHSV = new Scalar(105, 70, 70);
        Scalar highHSV = new Scalar(125, 110, 110);
        Mat thresh = new Mat();
        
        Core.inRange(mat, lowHSV, highHSV, thresh); // creates image with white as yellow, black as other colors
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300); // creates image with edges
        
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find contours of the image
        
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        ArrayList<Rect> boundRect = new ArrayList<Rect>();
        
        // uses contours to approximate polygon and then create bounding rectangles 
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect.add(Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray())));
        }
        
        double max = 0;
        int j = -1;
        
        for (int i = 0; i < boundRect.size(); i++) {
            if (boundRect.get(i).area() > max) {
                max = boundRect.get(i).area();
                j = i;
            }
        }
        
        int width = 176; // width of the webcam
        double center_x = width * 0.5;
        
        String position = "";
        
        if (j != -1) {
            Imgproc.rectangle(input, boundRect.get(j), new Scalar(255, 255, 255), 10); // drawing a white rectangle where the team element is
            double center = (boundRect.get(j).br().x + boundRect.get(j).tl().x) * 0.5;
            
            if (center < center_x) { 
                position = "left";
            } else if (center > center_x) {
                position = "center";
            } 
            
         
        } else {
            position = "right";
        }
        
        return input;
        
    }
    
    public String getAnalysis() {
        return position;
    }
}
}


