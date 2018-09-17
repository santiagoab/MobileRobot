#include <stdio.h>
#include <stdlib.h>
#include "Aria.h"
#include <iostream>
#include <math.h>

ArRobot robot;
ArSensorReading *sonarReading[8];
ArLaser *myLaser;

// The camera (Cannon VC-C4).
ArVCC4 vcc4(&robot);
// ACTS, for tracking blobs of color.
ArACTS_1_2 acts;

//Pointer of ArGripper 
ArGripper *myGripper = NULL;


//Variables declarations
double angle1, angle2, angle3, angle4, angle5;
double xpos, ypos, range, xps, yps, xrs, yrs, xpl, ypl, xrl, yrl;
double dist1, dist2, dist3, dist4, dist5;
double laserReading[9], laserTh[9];  
double gxs[8], gys[8], gxl[9], gyl[9]; // gxs & gys global sonar positions | gxl & gyl global laser positions
double d2r = 3.14 / 180;
int stage = 1;
int v1 = 150;         //Wheel velocities
int v2 = 100;
int v3 = 50;
int obst1 = 500;      //Distances to obstacles
int obst2 = 750; 
int obst3 = 1150;
int obst4 = 1300;
int obst5 = 1600;
int obst6 = 1900;


FILE *fpData;

class Chase
{
public:
	// Constructor.
	Chase(ArRobot *robot, ArACTS_1_2 *acts);
	// Destructor.
	~Chase(void);
	// The chase action.
	void ChaseAction();
	// Height and width of pixels from frame-grabber.
	enum {
		WIDTH = 160,
		HEIGHT = 120
	};
protected:
	ArRobot *myRobot;
	ArACTS_1_2 *myActs;
	int myChannel;
};

// Constructor: Initialize the chase action.
Chase::Chase(ArRobot *robot, ArACTS_1_2 *acts)
{
	myRobot = robot;
	myActs = acts;
	myChannel = 1;
}

// Destructor.
Chase::~Chase(void) {}

// The chase action.
void Chase::ChaseAction()
{
	ArACTSBlob blob;
	ArACTSBlob largestBlob;
	int numberOfBlobs;
	int blobArea = 10;
	double xRel, yRel;
	bool flag = false;
	numberOfBlobs = myActs->getNumBlobs(myChannel);

	// Get largest blob.
	if (numberOfBlobs != 0)
	{
		for (int i = 0; i < numberOfBlobs; i++)
		{
			myActs->getBlob(myChannel, i + 1, &blob);
			if (blob.getArea() > blobArea)
			{
				flag = true;
				blobArea = blob.getArea();
				largestBlob = blob;
			}
		}
	}

	if (flag == true)
	{
		// Determine where the largest blob's center of gravity is relative to the center of the camera and adjust xRel.
		xRel = (double)(largestBlob.getXCG() - WIDTH / 2.0) / (double)WIDTH - 0.15;
		yRel = (double)(largestBlob.getYCG() - HEIGHT / 2.0) / (double)HEIGHT;

		// Set the heading and velocity for the robot.
		if (ArMath::fabs(xRel) < .1)
		{
			myRobot->setDeltaHeading(0);
		}
		else
		{
			if (ArMath::fabs(xRel) <= 1)
				myRobot->setDeltaHeading(-xRel * 5);
			else if (-xRel > 0)
				myRobot->setDeltaHeading(5);
			else
				myRobot->setDeltaHeading(-5);
		}
		myRobot->setVel(60);
	}
}


// Use the Chase class defined above to declare an object named chase.
Chase chase(&robot, &acts);

void update(void);
ArGlobalFunctor updateCB(&update);

void update(void)
{
	
	// Store all sonar readings in array for further use.
	for (int a = 0; a < 8; a++) {
		sonarReading[a] = robot.getSonarReading(a);
	}
		
	// Get laser readings for navigation, divided into 5 equal sections.
	dist1 = myLaser->currentReadingPolar(-90, -54, &angle1);
	dist2 = myLaser->currentReadingPolar(-54, -18, &angle2);
	dist3 = myLaser->currentReadingPolar(-18, 18, &angle3);
	dist4 = myLaser->currentReadingPolar(18, 54, &angle4);
	dist5 = myLaser->currentReadingPolar(54, 90, &angle5);

	printf("Stage: %.2d ", stage);

	//For a better data representation and more accurate navigation in certain stages, we also store the laser readings divided into 9 equal 20 degree sections.
	laserReading[0] = myLaser->currentReadingPolar(-90, -70, &laserTh[0]);
	laserReading[1] = myLaser->currentReadingPolar(-70, -50, &laserTh[1]);
	laserReading[2] = myLaser->currentReadingPolar(-50, -30, &laserTh[2]);
	laserReading[3] = myLaser->currentReadingPolar(-30, -10, &laserTh[3]);
	laserReading[4] = myLaser->currentReadingPolar(-10, 10, &laserTh[4]);
	laserReading[5] = myLaser->currentReadingPolar(10, 30, &laserTh[5]);
	laserReading[6] = myLaser->currentReadingPolar(30, 50, &laserTh[6]);
	laserReading[7] = myLaser->currentReadingPolar(50, 70, &laserTh[7]);
	laserReading[8] = myLaser->currentReadingPolar(70, 90, &laserTh[8]);


	// Open a file to store sonar, laser and odometry readings.
	fpData = fopen("Data.txt", "a");
	if (fpData == NULL)
		std::cout << "File cannot be opened." << std::endl;

	// Get odometry readings
	// Get wheels velocity reading and display it.
	double leftV = robot.getLeftVel();
	double rightV = robot.getRightVel();

	fprintf(fpData, "%.2f\t%.2f\t", leftV, rightV);
	printf("Wleft %.2f Wright: %.2f", leftV, rightV);
	
	//Get global robot position and display it.
	double posX = robot.getX();
	double posY = robot.getY();

	fprintf(fpData, "%.2f\t%.2f\t", posX, posY);
	
	//Data calculations
	//Sonar data from sensor frame to robot frame, and from robot frame to the global frame.
	for (int a = 0; a < 8; a++) {
		range = sonarReading[a]->getRange();
		xps = range * cos(sonarReading[a]->getSensorTh() * d2r) + sonarReading[a]->getSensorX();
		yps = range * sin(sonarReading[a]->getSensorTh() * d2r) + sonarReading[a]->getSensorY();

		xrs = xps*cos(robot.getTh() * d2r) - yps*sin(robot.getTh() * d2r);
		yrs = xps*sin(robot.getTh() * d2r) + yps*cos(robot.getTh() * d2r);

		//Store final data into array and print in data file.
		gxs[a] = xrs + posX;
		gys[a] = yrs + posY;

		fprintf(fpData, "%.2f\t%.2f\t", gxs[a], gys[a]);

	}

	//Laser data from sensor frame to robot frame, and from robot frame to the global frame.
	for (int b = 0; b < 9; b++) {
		xpl = laserReading[b] * cos(laserTh[b] * d2r) + myLaser->getSensorPositionX();
		ypl = laserReading[b] * sin(laserTh[b] * d2r) + myLaser->getSensorPositionY();

		xrl = xpl*cos(robot.getTh() * d2r) - ypl*sin(robot.getTh() * d2r);
		yrl = xpl*sin(robot.getTh() * d2r) + ypl*cos(robot.getTh() * d2r);

		//Store final data into array and print in data file.
		gxl[b] = xrl + posX;
		gyl[b] = yrl + posY;

		fprintf(fpData, "%.2f\t%.2f\t", gxl[b], gyl[b]);

	}
	fprintf(fpData, "\n");

	//Robot movement through stages
	if (stage == 1)
	{
		robot.setVel2(v1, v1);                // Set starting velocity of the wheels.

		if (laserReading[4] < obst6)           //When detects front wall, input next stage.
		{
			stage = 2;
		}
		else if (laserReading[7]<520)         //Read distance to wall at 50-70 degrees and keep the robot
		{
			robot.setVel2(v2, 90);           //between 520 mm and 550 mm of distance from the wall, so the robot follows the wall.
		}
		else if (laserReading[7]>550)    
		{
			robot.setVel2(90, v2);
		}
	}

	if (stage == 2)
	{
		robot.setVel2(v1, 110);              //Turn right to go in between first two pillars T1 and T2

		if (laserReading[1] < 600 && laserReading[7] < 600)     //When robot goes between two pillars and detects them at 50-70 degrees at both sides, input next stage.
		{
			stage = 3;
		}
	}

	else if (stage == 3)
	{
		robot.setVel2(v1, v1);                // Go straight.

		if (dist3 < obst5)                    //When laser detects front wall, input next stage.
		{
			stage = 4;
		}
	}
	else if (stage == 4)
	{
		robot.setVel2(v1, v3);               //Turn right to go between pillars T3 and T4

		if (laserReading[2] < 850 && laserReading[6] < 850)   // When robot detects both obstacles at 50 - 70 degrees at both sides, input next stage.
		{
			stage = 5;
		}
	}
	else if (stage == 5)
	{
		robot.setVel2(v1, v1);               // Go straight, between obstacles T3 and T4.

		if (dist1 < 600 && dist5 < 600)      //When laser detects obstacles T3 and T4 at both sides, input next stage.
		{
			stage = 6;
			myGripper->gripOpen();           //Open gripper once
			myGripper->liftDown();           //Bring down the gripper
		}
	}
	else if (stage == 6)
	{          
		chase.ChaseAction();
		printf("GRIPPER:   %d\n", myGripper->getBreakBeamState());

		if (myGripper->getBreakBeamState() == 2)                  
		{
			myGripper->gripClose();
			myGripper->liftUp();
			stage = 7;
		}
	}
	else if (stage == 7)
	{
		robot.setVel2(v2, v1);              //Turn left.

	    if (dist3 < 650)             //When robot gets close to final goal front wall, stop wheels and stop robot.
	    {
		robot.setVel2(0, 0);
		if(myGripper->getGripState() == 2)
		{
			myGripper->gripOpen();
		}
		robot.stop();
		stage == 8;
	    }
		else if (dist3 < obst3)                  //When laser detects final goal front wall, go straight to it.
		{
			robot.setVel2(v1, v1);         
			
		}
	}
	else if (stage == 8)
	{
		exit(0);
	}

	fclose(fpData);	// Close the file.
}



int main(int argc, char **argv)
{
	// Initialisation
	Aria::init();

	// Open a connection to ACTS.
	acts.openPort(&robot);
	// Initialize the camera.
	vcc4.init();
	// Wait for a little while.
	ArUtil::sleep(2000);

	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	ArRobotConnector robotConnector(&argParser, &robot);
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

	// Always try to connect to the first laser:
	argParser.addDefaultArgument("-connectLaser");

	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (argParser.checkHelpAndWarnUnparsed())
		{
			// -help not given, just exit.
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	// Trigger argument parsing
	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
	}

	// Add sonar.
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	// Connect laser.
	if (!laserConnector.connectLasers())
	{
		ArLog::log(ArLog::Terse, "Could not connect to configured laser.");
		Aria::logOptions();
		Aria::exit(1);
	}
	myLaser = robot.findLaser(1);

	myGripper = new ArGripper(&robot);

	ArPose space(3800, 3500, 270); // Initial robot's odometry.
	robot.moveTo(space); //Moves the robot's idea of its position to this position.

						 // Tilt the camera down 45 degrees to make it find the ball easier.
	vcc4.tilt(-45);
	ArUtil::sleep(1000);

	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();

	robot.addUserTask("update", 50, &updateCB);
	//robot.setCycleTime(100);
	robot.runAsync(true);
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();

	Aria::exit(0);
}