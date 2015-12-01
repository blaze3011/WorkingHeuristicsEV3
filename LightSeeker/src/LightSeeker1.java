// --- Brief description
// --- LightSeeker1 --- (greedy)
// - Repeat until finished
//    - Make random motor action
//    - If the action decreases light, backtrack
// --- History
// 12/03/15. LightSeeker1 implemented. See "Versions.txt"
// 12/03/15. Initial testing. Simplistic and ineffective. Lots of back & forth. Excessively stuck. 


import java.io.IOException;
import java.util.Random;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.device.NXTMMX;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.video.Video;
import lejos.robotics.EncoderMotor;
import lejos.robotics.RegulatedMotor;
//import lejos.utility.Delay;
//import lejos.utility.Delay;

public class LightSeeker1 {

	private static final int WIDTH = 160;
    private static final int HEIGHT = 120;
    //private static final int NUM_PIXELS = WIDTH * HEIGHT;
    
    // Frames and motion maps
    private static byte [][] luminanceFrame = new byte[HEIGHT][WIDTH];
    //change this to 70 if it doesn't work then switch the fields
    private static int threshold = 70;
    //private static MotionMap aMotMap = new MotionMap();
    // Motors
    //private static RegulatedMotor motorB = new EV3LargeRegulatedMotor(MotorPort.B);
    private static EncoderMotor motorB = new UnregulatedMotor(MotorPort.B);
    //private static RegulatedMotor motorC = new EV3LargeRegulatedMotor(MotorPort.C);
	private static EncoderMotor motorC = new UnregulatedMotor(MotorPort.C);
	//private static float alpha = 180; // amplification for motor signals
	// Light features
	private static LightFeatures aLightFeat = new LightFeatures();
	private static double oldMeanLight, newMeanLight;
	// Randomness
	//private static Random randGenerator = new Random();
	private static int randDegLeft, randDegRight;
	private static int randMotor; // 0 = left; 1 = right
	
    public LightSeeker1() {
    	// Various initializations
    	randDegLeft = 0;
    	randDegRight = 0;
		// Initialize luminance frame
    	for (int x=0; x<WIDTH; x += 1) {
    		for (int y=0; y<HEIGHT; y += 1) {
    			luminanceFrame[y][x] = 0;
    		}
    	}
	}
    
    public static void main(String[] args) throws IOException  {
         
        EV3 ev3 = (EV3) BrickFinder.getLocal();
        Video video = ev3.getVideo();
        video.open(WIDTH, HEIGHT);
        byte[] frame = video.createFrame();
        double mot_amplif_larger = 1.2*0.6;
        double mot_amplif_smaller = 1.2*0.3;
        double left_field = 0;
        double right_field = 0;
//        EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S1);
        EV3ColorSensor evColour = new EV3ColorSensor(SensorPort.S2);
        
        int state = 0;
        
        // Grab frame
        video.grabFrame(frame);
    	// Extract luminanceFrame
        extractLuminanceValues(frame);
    	// Compute light features
        aLightFeat.compLeftRight(luminanceFrame, HEIGHT, WIDTH);
        //oldMeanLight = aLightFeat.meanTot;
    	 
        while(Button.ESCAPE.isUp()) {
        	
        	// --- Get webcam information
        	// Grab frame
        	video.grabFrame(frame);
        	// Extract luminanceFrame
        	extractLuminanceValues(frame);
        	// Compute light features
        	aLightFeat.compLeftRight(luminanceFrame, HEIGHT, WIDTH);
        	
        	//to store the distance values 
        	float[] onDist = new float[10];
        	
        	//to store light values
        	float[] sample = new float[10];
        	
        	//to fetch colour of the light values
        	evColour.getAmbientMode().fetchSample(sample, 0);

        	
        	// Display
//        	System.out.println("Mean Total: " + aLightFeat.meanTot);
//        	System.out.println("Mean left: " + aLightFeat.meanLeft);
//        	System.out.println("Mean right: " + aLightFeat.meanRight);
//        	dispFrame();
        	
        	// Regulated motors
        	//motorB.rotate((int) (mot_amplif*(aLightFeat.meanRight/255)*180), true); // true is for immediate return -> to parallelize motors
        	//motorC.rotate((int) (mot_amplif*(aLightFeat.meanLeft/255)*180));
        	// Unregulated motors
        	
        	right_field = (aLightFeat.meanRight/255)*180;
        	left_field = (aLightFeat.meanLeft/255)*180;
        	
        	//Ultrasonic sensor here
//    			ultraSensor.getDistanceMode().fetchSample(onDist, 0);
//    			//if distance is less than 0.2
//    			if(onDist[0] < 0.20){
//    				motorB.stop();
//        			motorC.stop();
//    			}
        	
        	//colour sensor code here uncomment to use
        	if(sample[0] <= 0.05) {
        		//very dark look for light
        		state = 0;
        		
        	} else if(sample[0] <= 0.05 && right_field < 50 && left_field < 50){
        		state = 1;
        	} 
        	else if(sample[0] > 0.20 && sample[0] < 0.26){
        		state = 3;
        	}else{
        		state = 2; 
        	}
        	
        	//try or here maybe
        	//try increase the number here to 70 or something
        	
        	if (state == 0){
        		//very dark
        		right_field = right_field + 60;
        		left_field = left_field + 60;
        		motorB.forward();
        		motorC.forward();
        	} else if(state == 1){
        		right_field = right_field + 75;
        	} else if(state == 2){
        		//normal light seeking
        		right_field = right_field + 75;
        	} else if(state ==3){
        		motorB.stop();
        		motorC.stop();
//        		makeSound.twoBeeps();
        	} else{
        		//error
        		System.out.println("Error");
        	}
        	
        	if (right_field > left_field) {
        		left_field = right_field * mot_amplif_larger;
        		right_field = left_field * mot_amplif_smaller;
        	} else {
        		right_field = right_field * mot_amplif_larger;
        		left_field = left_field * mot_amplif_smaller;
        	}
        	// B = left motor
        	// C = right motor
        	motorC.setPower((int) (right_field/2)); 
        	motorB.setPower((int) (left_field/2)); 
        	motorB.forward();
        	motorC.forward();
        	
        	/**
        	// Compute a random move
        	randDegLeft = randGenerator.nextInt(181);
        	randDegRight = randGenerator.nextInt(181);  	
        	// Make move
        	motorB.rotate(randDegRight, true); // true is for immediate return -> to parallelize motors
        	motorC.rotate(randDegLeft);
        	// Grab frame
        	video.grabFrame(frame);
        	// Extract luminanceFrame
        	extractLuminanceValues(frame);
        	// Compute light features
        	aLightFeat.compLeftRight(luminanceFrame, HEIGHT, WIDTH);
        	newMeanLight = aLightFeat.meanTot;
        	// Display mean light
        	System.out.println("Mean light: " + newMeanLight);
        	// If mean light has decreased, backtrack
        	if (newMeanLight < oldMeanLight) {
        		motorB.rotate(-randDegRight, true); // true is for immediate return -> to parallelize motors
            	motorC.rotate(-randDegLeft);
        	} else {
        		oldMeanLight = newMeanLight;
        	}
        	**/
//        	dispFrame();
        	        	        
        }
        video.close();
    }
    
    // DO: Improve this possibly by combining with chrominance values.
    public static void extractLuminanceValues(byte [] frame) {
    	int x,y;
    	int doubleWidth = 2*WIDTH; // y1: pos 0; u: pos 1; y2: pos 2; v: pos 3.
    	int frameLength = frame.length;
    	for(int i=0;i<frameLength;i+=2) {
    		x = (i / 2) % WIDTH;
    		y = i / doubleWidth;
    		luminanceFrame[y][x] = frame[i];
    	}
    }
    
    public static void dispFrame() {
    	for (int y=0; y<HEIGHT; y++) {
    		for (int x=0; x<WIDTH; x++) {
    			if (luminanceFrame[y][x] <= threshold) {
    				//make it follow these one values
    				//it means it is light for threshold of 5
    				LCD.setPixel(x, y, 1);
    			}
    			else {
    				//this means it's dark
    				LCD.setPixel(x, y, 0);
    			}	
    		}
    	}
    	
    }
    

}


