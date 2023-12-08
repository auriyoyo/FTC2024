package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.imgproc.Imgproc;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import java.util.*;
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

public class AutoBlue extends LinearOpMode {
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
        
         
        switch (snapshotAnalysis)
            {
                case "left":        
                {
                    telemetry.addLine("left");
                    break;
                }
                
                case "center":        
                {
                    telemetry.addLine("center");
                    break;
                }
                
                case "right":        
                {
                    telemetry.addLine("right");
                    break;
                }
            }
        telemetry.update();
            
        
    }
    public class Vision extends OpenCvPipeline {

    Mat mat = new Mat();
    Mat thresh = new Mat();
    Mat edges = new Mat();
    Mat hierarchy = new Mat();
    
    
    String position; 
    String max;
    String min;

    @Override
    public Mat processFrame(Mat input) {
        /*
        
                Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV); // creates HSV image
Imgproc.GaussianBlur(input, input, new Size(15, 15), 0);


        // Calculate the center coordinates of the image
        int centerX = input.cols() / 2;
        int centerY = input.rows() / 2;

        // Draw a cross in the center of the image
        int crossSize = 20; // Size of the cross arms
        Scalar color = new Scalar(0, 0, 255); // Red color

        // Draw the horizontal line of the cross
        Imgproc.line(input, new Point(centerX - crossSize, centerY), new Point(centerX + crossSize, centerY), color, 2);

        // Draw the vertical line of the cross
        Imgproc.line(input, new Point(centerX, centerY - crossSize), new Point(centerX, centerY + crossSize), color, 2);


double[] min = new double[3];
min[0] = Double.MAX_VALUE;
double[] max = new double[3];
double[] curr = new double[3];


for (int r = centerX - crossSize; r < centerX + crossSize; r++) {

for (int c = centerY - crossSize; c < centerY + crossSize; c++) {

 curr = input.get(r,c);

if (min[0] > curr[0]) {

 min = curr;

}

if (max[0] < curr[0]) {

max = curr;

}
}
}






 Imgproc.putText(input, "" + (int)min[0] + "," + (int)min[1] + "," + (int)min[2], new Point(0, 12), 0, 0.5, new Scalar(255, 255, 255), 1);
 Imgproc.putText(input, "" + (int)max[0] + "," + (int)max[1] + "," + (int)max[2], new Point(75, 12), 0, 0.5, new Scalar(255, 255, 255), 1);

    
     telemetry.addLine("" + (int)min[0] + " " + (int)min[1] +" " + (int)min[2]  );
     telemetry.addLine("" + (int)max[0] + " " + (int)max[1] +" " + (int)max[2]  );
     telemetry.update();
      
         
         //RED
        //Scalar lowHSV = new Scalar(0, 155, 175);
        //Scalar highHSV = new Scalar(0, 195, 195);
        
        //Scalar lowHSV = new Scalar(1,135, 110); 
        //Scalar highHSV = new Scalar(34, 165,135);
        
        
        
        //BLUE
        //Scalar lowHSV = new Scalar(105, 70, 70);
        //Scalar highHSV = new Scalar(125, 110, 110);

        //Scalar lowHSV = new Scalar(100, 100, 50);
        //Scalar highHSV = new Scalar(125, 160, 145);

        //Scalar lowHSV = new Scalar(105,180, 105); 
        //Scalar highHSV = new Scalar(120, 215,120);
        
        return input;
        */
        
        
        
        
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // creates HSV image
        Imgproc.GaussianBlur(mat, mat, new Size(15, 15), 0);
        
        Scalar lowHSV = new Scalar(105,180, 105); 
        Scalar highHSV = new Scalar(120, 215,120);
        
        
      
        
        Core.inRange(mat, lowHSV, highHSV, thresh); // creates image with white as yellow, black as other colors
        
        Imgproc.Canny(thresh, edges, 100, 300); // creates image with edges
        
        List<MatOfPoint> contours = new ArrayList<>();
      
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
        
        int width = 1280; // width of the webcam
        double center_x = width * 0.5;
     
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



