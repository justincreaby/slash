/*******************************************************************************
* race.cpp
*
* Justin Creaby 2018
* This program runs an autonomous car around a predefined path.
* 
* The car will stop when a position of 0, 0 is desired or by hitting control C
* 
* Example of calling this function:
* 
*		sudo ./race 4 pathFile.txt
* 
*******************************************************************************/
extern "C" {
    #include <rc_usefulincludes.h>
	#include <roboticscape.h>
}

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include "encoderFunctions.h"
#include "readParameterAndPathFileToVector.h"
#include <algorithm>    // std::min
//#include "URG04LX.hpp"
//#include "../../BreezyLiDAR/cpp/URG04LX.cpp"
//#include "../../BreezyLiDAR/c/hokuyo.h"
//#include "../../BreezyLiDAR/c/hokuyo.c"//#include "../../BreezyLiDAR/cpp/URG04LX.cpp"

//static const char * DEVICE = "/dev/ttyACM0";


/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	rc_set_state(RUNNING);
	return;
}


int main(int argc, char *argv[]){
	if(argc != 4 && argc != 5)
	{
		printf("    Incorrect usage.\n");
		printf("    Need to specify amount of time (sec) to calibrate gyros.\n");
		printf("    Need to provide a parameter file.\n");
		printf("    Need to provide a path file.\n");
		printf("    Example: sudo ./race 4 parameterFile.txt pathFile.txt\n\n");
		printf("    When recording a path, use the following format\n");
		printf("    Example: sudo ./race 4 parameterFile.txt pathFile.txt recordedPathFilename.txt\n");
		return 0;
	}
	
	bool recordPath = false;
	if (argc == 5)
	{
		recordPath = true;
	}
	
	// Read in tunable parameters
	std::string parameterFileName = argv[2];
	std::vector<double> parameterValues;
	readParameterFile(parameterFileName, parameterValues);
	// for (int i=0; i<parameterValues.size(); i++)
 //   {
 //       std::cout << parameterValues[i] << "\n";
 //   }
    
    // Read in the path from a csv text file.
	std::string pathFileName = argv[3];
	std::vector<double> pathX, pathY;
	readPathFile(pathFileName, pathX, pathY);
    
	// Initialize LEDs
	rc_set_led(GREEN,OFF);
	rc_set_led(RED,ON);
	
	// Timing Variables
	double timeStep = 0.01f; // Time step of program, (seconds).
	uint64_t timingThreshold = (uint64_t) (timeStep * 1000000000.0);
	int timeStepCount = 0; // Used for printing out to the screen
	double timeElapsed = 0.0f; // Used for printing time passed
	uint64_t startTimerNanoSeconds; // Timer for the timing while loop
	int initializeCount = 0; // Counter for initializing the ECS motor and steering

	// Distance & Speed Variables
	bool   createLogFile		= parameterValues[0];
	double propGain 			= parameterValues[1];
	double intGain				= parameterValues[2];
	double straightSpeed		= parameterValues[3]; // any faster than 3.0 will cause it to wheel spin on gravel
	double curveSpeed			= parameterValues[4];
	double obstacleSpeed		= parameterValues[5];
	double velocityAverageTime	= parameterValues[6];
	double velocityRateLimit	= parameterValues[7];
	double metersPerPulseMotor	= parameterValues[8]; //6.9088/128; 18.288/151.0; //  Distance travelled between pulses.
	double lookAheadDistance	= parameterValues[9];
	double integratorError = 0.0;
	double lastVelSetpoint = straightSpeed/2.0;
	int encoderPos = rc_get_encoder_pos(1); // Current encoder position

	int encoderChannelMotor = 1;
	int oldPulseCountMotor = rc_get_encoder_pos(encoderChannelMotor); // Previous encoder position

	std::deque<double> distanceHistory ((int) (velocityAverageTime/timeStep),0); // Deque to hold the recent history of calculated distances

	// Initial Starting Position
	unsigned int pathIndex = 0; // Starting path index
	double xPosition = pathX[pathIndex]; // Initial X position
	double yPosition = pathY[pathIndex]; // Initial Y position
	double lastRecordedPositionX = xPosition;
	double lastRecordedPositionY = yPosition;
	
	// Steering / direction variables
	double steeringAngle = 0.0;
	double lidarSteeringAngle = 0.0;
	rc_imu_data_t data; // Struct to hold gyro data
	double gyroZBias; // Gyro Z bias
	int gyroCount = 1;
	double gyroCalTime = atof(argv[1]);
	int maxGyroCount = (int) (gyroCalTime / timeStep);
	int calcGyroBias = 1;
	double gyroHeading = 0.0; // Initial vehicle heading
	double steerBias = -0.125; // Steering bias, 0 degrees = -0.125 PWM
	double steeringSaturationAngle = 28.0; // degrees
	double steeringRateLimit = 40.0;
	double lastSteeringAngle = 0.0;
	double wheelBase = 0.333375; // 0.333375 m (13.125 inches)

	// Initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	// Initialize ESC motor and steering.
	printf("Initializing ESC motor & steering\n");
	while (initializeCount< (int) (0.1 / timeStep)) // Send commands for 0.1 seconds.
	{
		rc_send_servo_pulse_normalized(1, steerBias); // Steer straight
		rc_send_esc_pulse_normalized(2, 0.50); // Send neutral velocity
		initializeCount++;
		rc_usleep(timeStep*1000000);
	}
	rc_usleep(500000); // Pause for 0.5 second, so that any vibrations do not impact the gyro cal
	printf("Initializing motor and steering complete.\n");

	// Initialize IMU
	rc_imu_config_t conf = rc_default_imu_config();
	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}
	
	// Calibrate Gyro
	printf("Calibrating Gyro Bias\n");
	if(rc_read_gyro_data(&data)<0)
	{
		printf("read gyro data failed\n");
	}
	gyroZBias = data.gyro[2]; // Set initial bias to the current gyro reading.
	while (calcGyroBias)
	{
		if(rc_get_state()==EXITING)
		{
			return 0;
		}
		
		if(rc_read_gyro_data(&data)<0)
		{
			printf("read gyro data failed\n");
		};
		gyroZBias = gyroZBias+(data.gyro[2]-gyroZBias)/gyroCount;
	
		gyroCount++;
		if (gyroCount >= maxGyroCount)
		{
			calcGyroBias = 0;
		}
		rc_usleep(timeStep*1000000);
		printf("Elapsed time: %f Calibrating Gyro Bias: Z Bias = %f\n", gyroCount*timeStep, gyroZBias);
	}
	printf("Elapsed time: %f Calibrating Gyro Bias: Z Bias = %f\n", gyroCount*timeStep, gyroZBias);
		
	// Set state to paused, stay in this loop until pause button is pressed
	printf("Press PAUSE button to start race.\n");
	rc_set_pause_released_func(&on_pause_released);
	rc_set_led(GREEN,ON);
	rc_set_led(RED,OFF);
	rc_set_state(PAUSED);
	while(rc_get_state()==PAUSED)
	{
		rc_usleep(100000);
	}
	printf("Pause was pressed, starting race.\n");
	rc_set_led(GREEN,OFF);
	rc_set_led(RED,ON);
	
	// Write to csv text file
	std::ofstream logFile;
	if (createLogFile)
	{
    	logFile.open ("logFile.csv");
    	logFile << "timeElapsed, oldPulseCountMotor\n";
	}
	printf("timeElapsed, oldPulseCountMotor, pathIndex, pathX[ii], pathY[ii], steeringAngle, gyroHeading, xPosition, yPosition, velocity, velSetpoint, ESCCommand, integratorError\n");
	
	// Record path in .txt file
	std::ofstream pathFile;
	if (recordPath)
	{
    	pathFile.open (argv[4]);
    	pathFile << xPosition << ", " << yPosition << ", " << obstacleSpeed << std::endl;
	}
	
	// Starting main loop
	while(rc_get_state()!=EXITING)
	{
		startTimerNanoSeconds = rc_nanos_since_boot();

		// Get gyro data and update heading
		rc_read_gyro_data(&data);
		gyroHeading += (data.gyro[2] - gyroZBias)*timeStep*1.0; // 1.0125 worked ok for first runs. 1.005 works well, turns inside a little. This gain is needed when the time of the loop is not quite real time.
		// I believe the gyro is under measuring and thus over turning

		// Get encoder value and update x/y position
		double motorEncoderDistance = calcDistanceTravelled(oldPulseCountMotor, metersPerPulseMotor, encoderChannelMotor);
		distanceHistory.pop_back(); // Remove from back of encoder history deque

			distanceHistory.push_front (motorEncoderDistance); // Add new distance travelled to deque. Used for velocity calculation
			double SIncrement = motorEncoderDistance*(sin(gyroHeading*0.0174532925199)); // 2.0*PI/360.0 =  0.0174532925199
			double CIncrement = motorEncoderDistance*(cos(gyroHeading*0.0174532925199));
			// if (reverse)
			// {
			// 	// distanceIncrement is opposite in reverse
			// 	xPosition = xPosition+SIncrement;
			// 	yPosition = yPosition-CIncrement;
			// }
			// else
			// {
				xPosition = xPosition-SIncrement;
				yPosition = yPosition+CIncrement;
			// }


		
		// Calculate Velocity from Encoder Pulses
		double distanceHistorySum = 0;
		for (unsigned int ii = 0; ii<distanceHistory.size(); ii++)
		{
			distanceHistorySum += distanceHistory.at(ii);
		}
		double velocity = distanceHistorySum / velocityAverageTime;
		
		// Determine if car is stuck
		// if (velocity == 0 && reverse == 0)
		// {
		// 	stuckCount++;
		// 	if (stuckCount > (int) (2.0 / timeStep) ) // Put car in reverse after 1 second of being stuck
		// 	{
		// 		reverse = 1;
		// 		forward = 0;
		// 		stuckCount = 0;
		// 	}
		// 	//printf("stuck, stuckCount = %d, (int) (1.0 / timeStep) = %d\n", stuckCount, (int) (1.0 / timeStep));
		// }
		// else
		// {
		// 	stuckCount = 0;
		// }

		// Determine Goal Point
		// While within 1 meter of the desired point, increase the index of desired point
        while (sqrt((pathX[pathIndex] - xPosition)*(pathX[pathIndex] - xPosition) + (pathY[pathIndex] - yPosition)*(pathY[pathIndex] - yPosition)) < lookAheadDistance)
        {
            pathIndex++;
			if (pathIndex >= pathX.size() - 1)
			{
				printf("\n1st break\n");
				break;
			}
        }
	
		if (pathIndex >= pathX.size() - 1)
    	{
    	  	printf("2nd break\n");
    	  	rc_send_servo_pulse_normalized(1, steerBias);
    	  	break;
    	}	

		// VELOCITY SETPOINT & ESC COMMAND (SPEED CONTROL) CALCULATION //
		double velSetpoint;
		// if ( (pathIndex >= 26 && pathIndex <= 60) || (pathIndex >= 96 && pathIndex <= 128) )
		// //if (pathIndex >= 3 && pathIndex <= 11)
		// {
		//  	velSetpoint = curveSpeed; // MEDIUM
		//  	if ( (pathIndex >= 40 && pathIndex <= 49) || (pathIndex >= 99 && pathIndex <= 128) )// || (pathIndex >= 120 && pathIndex <= 128) )
		//  	//if (pathIndex >= 6 && pathIndex <= 11)
		//  	{
		//  		velSetpoint = obstacleSpeed; // SLOW
		//  	}
		// }
		// else
		// {
			 velSetpoint = straightSpeed; // FAST
		// }
	
		// Rate limit the velocity
    	double velRateLim_timeStep = velocityRateLimit * timeStep;
    	if ((velSetpoint - lastVelSetpoint) > velRateLim_timeStep)
    	{
    	    velSetpoint = lastVelSetpoint + velRateLim_timeStep;
    	}
    	else if ((-velSetpoint + lastVelSetpoint) > velRateLim_timeStep)
    	{
    	    velSetpoint = lastVelSetpoint - velRateLim_timeStep;
    	}
        lastVelSetpoint = velSetpoint;
		
		double eVelSetpoint = velSetpoint - velocity;
		integratorError = integratorError + eVelSetpoint * timeStep;
		double ESCCommand = eVelSetpoint * propGain + integratorError * intGain;
		if (ESCCommand < 0.0)
		{
			ESCCommand = 0.0;
			if (intGain>0.0)
			{
				integratorError = integratorError - eVelSetpoint * timeStep; // Anti-windup
			}
		}
		if (ESCCommand > 0.25)
		{
			ESCCommand = 0.25;
			if (intGain>0.0f)
			{
				integratorError = integratorError - eVelSetpoint * timeStep; // Anti-windup
			}
		}
		ESCCommand = ESCCommand + 0.5; // 0.5 = zero speed. Add this offset.
		
		// STEERING ANGLE CALCULATION //
		// Calculate the desired steering angle based on the Pure Pursuit method
    	double dE = pathX[pathIndex] - xPosition;
    	double dN = pathY[pathIndex] - yPosition;
    	double dist2DesiredPoint = sqrt(dE*dE + dN*dN);
    	double goalPointAngle = -1.0*atan2(dE,dN); // same as from matlab script: float matlabangle = -(PI/2-atan2(dN,dE));
		double alpha = goalPointAngle - gyroHeading*0.0174532925199;
    	double k = 2.0 * sin(alpha) / dist2DesiredPoint;

    	steeringAngle = atan(k*wheelBase) * 57.295779513082323 * (1.5);	// 360/(2*PI) = 57.295779513082323
    
    	// Saturate the steering angle
    	steeringAngle = (steeringAngle > steeringSaturationAngle) ? steeringSaturationAngle : steeringAngle; // Max
    	steeringAngle = (steeringAngle < -steeringSaturationAngle) ? -steeringSaturationAngle : steeringAngle; // Min

    	// Rate limit the steering angle
    	double strRateLim_timeStep = steeringRateLimit * timeStep;
    	if ((steeringAngle - lastSteeringAngle) > strRateLim_timeStep) 
    	{
    		steeringAngle = lastSteeringAngle + strRateLim_timeStep;
    	}
    	else if ((-steeringAngle + lastSteeringAngle) > strRateLim_timeStep)
    	{
    		steeringAngle = lastSteeringAngle - strRateLim_timeStep;
    	}
		lastSteeringAngle = steeringAngle;
    	
    	// Convert from steering angle to PWM
    	double steeringPWM = -0.0275*steeringAngle + steerBias; // steeringSlope = (0.68 - (-0.75)) / (-29 - (23)) = 1.43 / -52 = -0.0275 ; // (y2 - y1) / (x2 - x1)

		// Send steering annd velocity command
		// if (1);//(forward)
		// {
		 	rc_send_esc_pulse_normalized(2, ESCCommand);
		 	rc_send_servo_pulse_normalized(1, steeringPWM);
		// 	reverseSet = 0;
		// 	neutralSet = 0;
		// 	//printf("forward\n");
		// }
		
		// if (0);//(reverse)
		// {
		// 	//rc_send_esc_pulse_normalized(2, 0.42);
		// 	//rc_send_servo_pulse_normalized(1, steerBias); // Steer straight
		// 	reverseCount++;
		// 	if (reverseCount > (int) (2.0 / timeStep)) // Reverse for 1 second
		// 	{
		// 		reverseCount = 0;
		// 		forward = 1;
		// 		stuck = 0;
		// 		reverse = 0;
		// 	}
			
		// 	if (!reverseSet)
		// 	//send a backwards signal for 1 sample
		// 	{
		// 		rc_send_esc_pulse_normalized(2, 0.43);
		// 		reverseSet = 1;
		// 		//printf("back for 1 sample\n");
		// 	}
		// 	else if (!neutralSet)
		// 	{
		// 		rc_send_esc_pulse_normalized(2, 0.5);
		// 		neutralCount++;
		// 		if (neutralCount > (int) (0.1 / timeStep))
		// 		{
		// 			neutralSet = 1;
		// 			neutralCount = 0;
		// 		}
		// 		//printf("neutral for 0.1 seconds\n");
		// 	}
		// 	else{
		// 		rc_send_esc_pulse_normalized(2, 0.42);
		// 		//printf("backwards\n");
		// 	}
			
		// 	// Reset velocity control variables
		// 			        	printf("stuck vel setpoint = %f\n",velSetpoint );
  //      	printf("stuck last vel setpoint = %f\n",lastVelSetpoint );
		// 	lastVelSetpoint = 0;
		// 	integratorError = 0;
		// 	//printf("reverse\n");
		// 	//printf("reverse, reverseCount = %d, (int) (1.0 / timeStep) = %d\n", reverseCount, (int) (1.0 / timeStep));
		// }
		

		// Print to the screen
		if (timeStepCount % 500 == 0)
		{
		//printf("\r");
	    printf("%7.2f   | %8d | %8d | %10.2f | %10.2f | %10.2f | % 10.2f | % 10.2f | % 9.2f | % 8.2f | % 10.2f | % 10.2f | % 10.2f \n",timeElapsed, oldPulseCountMotor, pathIndex, pathX[pathIndex], pathY[pathIndex], steeringAngle, gyroHeading, xPosition, yPosition, velocity, velSetpoint, ESCCommand, integratorError);
		//fflush(stdout);
		}
		
		if (createLogFile)
		{
			logFile << timeElapsed << ", " << oldPulseCountMotor << std::endl;
		}
		//, yPosition, velocity, velSetpoint, ESCCommand, integratorError("%7.2f   | %8d | %10.2f | %10.2f | %10.2f | % 10.2f | % 10.2f | % 9.2f | % 8.2f | % 10.2f | % 10.2f | % 10.2f") << std::endl;
		
		
		// Record Path
		if (recordPath)
		{
			if (sqrt((lastRecordedPositionX - xPosition)*(lastRecordedPositionX - xPosition) + (lastRecordedPositionY - yPosition)*(lastRecordedPositionY - yPosition)) > 0.25)
			{
				pathFile << xPosition << ", " << yPosition << ", " << std::max(obstacleSpeed,velocity) << std::endl;
				lastRecordedPositionX = xPosition;
				lastRecordedPositionY = yPosition;
			}
		}
		
		timeElapsed += timeStep;
		timeStepCount++;
		
		// Stay in this loop until the amount of time for the loop has reached timeStep
		//while ( (rc_nanos_since_boot() - startTimerNanoSeconds) < ( (uint64_t) (timeStep * 1000000000)) )
		while ( (rc_nanos_since_boot() - startTimerNanoSeconds) < timingThreshold )
		{
			//printf("time Step time =  %lldns\n", rc_nanos_since_boot()-startTimerNanoSeconds);
			// Wait until 1/frequ milliseconds has elapsed
		}
		//printf("time Step time =  %lldns\n", rc_nanos_since_boot()-startTimerNanoSeconds);
		//printf("time Step =  %lldns\n", (uint64_t) (timeStep * 1000000000));
	}
	
	rc_send_servo_pulse_normalized(1, steerBias);
	rc_send_esc_pulse_normalized(2, 0.5); //Apply the brake at the end. Don't do this. Let the car roll to a stop instead to ensure it crosses the finish line.
	logFile.close();
	pathFile.close();
	rc_cleanup();
	return 0;
}
