/*
str2int function taken from http://stackoverflow.com/questions/16388510/evaluate-a-string-with-a-switch-in-c
*/

#include "GL/freeglut.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <cmath>
#include <ctime>
#include <conio.h>

using namespace std;
//menu variables
#define DEFINE_CIRCLE_OBSTACLE		1
#define DEFINE_LINE_OBSTACLE		2
#define DEFINE_CIRCLE_SIZE			7
#define SINGLE_STEP					3
#define EXIT						4
#define DELETE_OBSTACLE				5
#define CLEAR_ALL_OBSTACLES			8
#define RESTART						9

#define BOID_MAX					200
#define OBSTACLE_MAX				100
#define M_PI						3.14159265358979323846

//global variables//
int width = 1000;
int height = 1000;
double triangle_size = 0.05;
double step_size = 0.02;
double refresh_rate = 0.025;
double radius_i = 0.180;
double radius_m = 0.409;
double radius_o = 0.860;
double blind_degrees = 320;
double field_radius = 6.0;
double endPenRadius = 0.5;
int penCount = 0;
int restartFrameCount = 0;
int CIRCLE_OBSTACLE_NUMBER = 0;
int LINE_OBSTACLE_NUMBER = 0;
bool run = true;
int leftClicks = 0;
int define_mode = 0;
int BOID_NUMBER = 100;
double endGoalStrength = 30;
double turnAngle = 10;
int randomSeed = 123;

//global variables for windowTwo
int width2 = 500;
int height2 = 550;
int windowOne, windowTwo;
double slidersX[8];
int sliderSelected = 0;
bool movingSlider = false;
bool callWindowOne = false;
bool slidersInitialized = false;
bool load = false;
bool save = false;

struct Color {
	double red, green, blue;
};

struct Boid {
	double x;
	double y;
	double nextX;
	double nextY;
	double degrees;
	double nextDegrees;
	Color c;
	bool inPen;
};

Boid boids[BOID_MAX];
Boid centerBoid;
Boid endPoint;

struct CircleObstacle {
	Boid center;
	double size = 1.0;
	double repelStrength = 2.0 + (size / 3.0);
};

struct LineObstacle {
	Boid start;
	Boid end;
	double length = 0.0;
	double size = 0.5;
	double repelStrength = 4.0;
};

CircleObstacle circle_obstacles[OBSTACLE_MAX];
LineObstacle line_obstacles[OBSTACLE_MAX];
Boid closestPointOnLine;
bool closestToEdges = false;
double closestToEdgesDistance = 0.0;

//functions
static void init();
static void init2();
static void display();
static void display2();
double velocity(Boid boid);
double degreeAngle(Boid boid1, Boid boid2);
void processMenuEvents(int option);
double distance(Boid boid1, Boid boid2);
double distanceFromLine(Boid boid, LineObstacle lineObstacle);
bool affected(Boid boid, LineObstacle lineObstacle, double dist);

void createGLUTMenus() {
	//main menu
	int menu = glutCreateMenu(processMenuEvents);
	glutAddMenuEntry("Define Circle Obstacle", DEFINE_CIRCLE_OBSTACLE);
	glutAddMenuEntry("Define Line Obstacle", DEFINE_LINE_OBSTACLE);
	glutAddMenuEntry("Delete Obstacle", DELETE_OBSTACLE);
	glutAddMenuEntry("Delete All Obstacles", CLEAR_ALL_OBSTACLES);
	glutAddMenuEntry("Single-Step", SINGLE_STEP);
	glutAddMenuEntry("Restart", RESTART);
	glutAddMenuEntry("Exit", EXIT);

	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void processMenuEvents(int option) {
	switch (option) {
	case DEFINE_CIRCLE_OBSTACLE:
		CIRCLE_OBSTACLE_NUMBER++;
		define_mode = DEFINE_CIRCLE_OBSTACLE;
		break;
	case DEFINE_LINE_OBSTACLE:
		LINE_OBSTACLE_NUMBER++;
		define_mode = DEFINE_LINE_OBSTACLE;
		break;
	case DELETE_OBSTACLE:
		define_mode = DELETE_OBSTACLE;
		break;
	case CLEAR_ALL_OBSTACLES:
		while (CIRCLE_OBSTACLE_NUMBER > 0) {
			CIRCLE_OBSTACLE_NUMBER--;
			circle_obstacles[CIRCLE_OBSTACLE_NUMBER].size = 1;
		}
		while (LINE_OBSTACLE_NUMBER > 0) {
			LINE_OBSTACLE_NUMBER--;
		}
		leftClicks = 0;
		break;
	case SINGLE_STEP:
		printf("step = <SPACE>\nrun = <Enter>\n");
		run = false;
		break;
	case RESTART:
		penCount = 0;
		restartFrameCount = 0;
		init();
		break;
	case EXIT:
		exit(0);
	}
}

static void mouse_passive_move(int x, int y) {
	double widthd = width;
	double heightd = height;
	double pointx = (x / widthd) * field_radius * 2 - field_radius;
	double pointy = ((heightd - y) / heightd) * field_radius * 2 - field_radius;

	if (define_mode == DEFINE_CIRCLE_OBSTACLE) {
		circle_obstacles[CIRCLE_OBSTACLE_NUMBER - 1].center.x = pointx;
		circle_obstacles[CIRCLE_OBSTACLE_NUMBER - 1].center.y = pointy;
	}
	else if (define_mode == DEFINE_LINE_OBSTACLE && leftClicks == 0) {
		line_obstacles[LINE_OBSTACLE_NUMBER - 1].start.x = pointx;
		line_obstacles[LINE_OBSTACLE_NUMBER - 1].start.y = pointy;
		line_obstacles[LINE_OBSTACLE_NUMBER - 1].end.x = pointx;
		line_obstacles[LINE_OBSTACLE_NUMBER - 1].end.y = pointy;
	}
	else if (define_mode == DEFINE_LINE_OBSTACLE && leftClicks == 1) {
		line_obstacles[LINE_OBSTACLE_NUMBER - 1].end.x = pointx;
		line_obstacles[LINE_OBSTACLE_NUMBER - 1].end.y = pointy;
		line_obstacles[LINE_OBSTACLE_NUMBER - 1].length = distance(line_obstacles[LINE_OBSTACLE_NUMBER - 1].start, line_obstacles[LINE_OBSTACLE_NUMBER - 1].end);
		line_obstacles[LINE_OBSTACLE_NUMBER - 1].start.nextDegrees = degreeAngle(line_obstacles[LINE_OBSTACLE_NUMBER - 1].start, line_obstacles[LINE_OBSTACLE_NUMBER - 1].end);
		if (line_obstacles[LINE_OBSTACLE_NUMBER - 1].start.nextDegrees > 180) {
			line_obstacles[LINE_OBSTACLE_NUMBER - 1].start.nextDegrees -= 180;
		}
	}

	display();
}

static void mouse_active_move(int x, int y) {
	double widthd = width;
	double heightd = height;
	double pointx = (x / widthd) * field_radius * 2 - field_radius;
	double pointy = ((heightd - y) / heightd) * field_radius * 2 - field_radius;

	if (define_mode == DEFINE_CIRCLE_SIZE) {
		Boid pos;
		pos.x = pointx;
		pos.y = pointy;
		circle_obstacles[CIRCLE_OBSTACLE_NUMBER - 1].size = 3.0 * distance(circle_obstacles[CIRCLE_OBSTACLE_NUMBER - 1].center, pos);
		circle_obstacles[CIRCLE_OBSTACLE_NUMBER - 1].repelStrength = 2.0 + circle_obstacles[CIRCLE_OBSTACLE_NUMBER - 1].size;
	} 

	display();
}

static void mouse_press(int button, int state, int x, int y) {
	double widthd = width;
	double heightd = height;
	double pointx = (x / widthd) * field_radius * 2 - field_radius;
	double pointy = ((heightd - y) / heightd) * field_radius * 2 - field_radius;

	if (button == 0 && state == 0) {
		if (define_mode == DEFINE_CIRCLE_OBSTACLE) {
			define_mode = DEFINE_CIRCLE_SIZE;
		}
		else if (define_mode == DEFINE_LINE_OBSTACLE && leftClicks == 0) {
			leftClicks++;
		}
		else if (define_mode == DEFINE_LINE_OBSTACLE && leftClicks == 1) {
			define_mode = 0;
			leftClicks = 0;
		}
		else if (define_mode == DELETE_OBSTACLE) {
			leftClicks = 0;
			Boid pos;
			pos.x = pointx;
			pos.y = pointy;
			bool deleted = false;
			for (int i = 0; i < CIRCLE_OBSTACLE_NUMBER; i++) {
				if (deleted) {
					if (i != CIRCLE_OBSTACLE_NUMBER - 1) {
						circle_obstacles[i] = circle_obstacles[i + 1];
					}
					else {
						CIRCLE_OBSTACLE_NUMBER--;
						return;
					}
				}
				else if (distance(pos, circle_obstacles[i].center) < circle_obstacles[i].size / 3.0) {
					deleted = true;
					circle_obstacles[i] = circle_obstacles[i + 1];
					if (i == CIRCLE_OBSTACLE_NUMBER - 1) {
						CIRCLE_OBSTACLE_NUMBER--;
						return;
					}
				}
			}
			for (int i = 0; i < LINE_OBSTACLE_NUMBER; i++) {
				double dist = distanceFromLine(pos, line_obstacles[i]);
				if (deleted) {
					if (i != LINE_OBSTACLE_NUMBER - 1) {
						line_obstacles[i] = line_obstacles[i + 1];
					}
					else {
						LINE_OBSTACLE_NUMBER--;
						return;
					}
				}
				else if (dist < line_obstacles[i].size / 10.0 && affected(pos, line_obstacles[i], dist)) {
					deleted = true;
					line_obstacles[i] = line_obstacles[i + 1];
					if (i == LINE_OBSTACLE_NUMBER - 1) {
						LINE_OBSTACLE_NUMBER--;
						return;
					}
				}
			}
		}
	}
	if (button == 0 && state == 1) {
		if (define_mode == DEFINE_CIRCLE_SIZE) {
			define_mode = 0;
		}
	}
	display();
}

void updateParameter() {
	double left = -0.25;
	double right = 0.7;
	double percentage = (slidersX[sliderSelected] - left) / (right - left);

	switch (sliderSelected) {
	case 0: { //Boid Number
		double max = BOID_MAX;
		double min = 1;
		BOID_NUMBER = (int)((max - min) * percentage + min);
		init();
		init2();
	}
		break;
	case 1: { //Radius Repulsion
		double max = 0.5;
		double min = 0.01;
		radius_i = ((max - min) * percentage + min);
	}
		break;
	case 2: { //Radius Orientation
		double max = 0.9;
		double min = 0.01;
		radius_m = ((max - min) * percentage + min);
	}
		break;
	case 3: { //Radius Attraction
		double max = 1.5;
		double min = 0.01;
		radius_o = ((max - min) * percentage + min);
	}
		break;
	case 4: { //Time Step
		double max = 0.06;
		double min = 0.001;
		step_size = ((max - min) * percentage + min);
	}
		break;
	case 5: { //Perception Angle
		double max = 360;
		double min = 1;
		blind_degrees = (int)((max - min) * percentage + min);
	}
		break;
	case 6: { //Turn Angle
		double max = 20;
		double min = 1;
		turnAngle = ((max - min) * percentage + min);
	}
		break;
	case 7: { //Strength of Goal
		double max = 50;
		double min = 1;
		endGoalStrength = ((max - min) * percentage + min);
	}
		break;
	}
}

void initializeSliderPositions() {
	double left = -0.25;
	double right = 0.7;

	for (int i = 0; i < 8; i++) {
		switch (i) {
		case 0: { //Boid Number
			double max = BOID_MAX;
			double min = 1;
			double percentage = BOID_NUMBER / (max - min);
			slidersX[0] = percentage * (right - left) + left;
		}
			break;
		case 1: { //Radius Repulsion
			double max = 0.5;
			double min = 0.01;
			double percentage = radius_i / (max - min);
			slidersX[1] = percentage * (right - left) + left;
		}
			break;
		case 2: { //Radius Orientation
			double max = 0.9;
			double min = 0.01;
			double percentage = radius_m / (max - min);
			slidersX[2] = percentage * (right - left) + left;
		}
			break;
		case 3: { //Radius Attraction
			double max = 1.5;
			double min = 0.01;
			double percentage = radius_o / (max - min);
			slidersX[3] = percentage * (right - left) + left;
		}
			break;
		case 4: { //Time Step
			double max = 0.06;
			double min = 0.001;
			double percentage = step_size / (max - min);
			slidersX[4] = percentage * (right - left) + left;
		}
			break;
		case 5: { //Perception Angle
			double max = 360;
			double min = 1;
			double percentage = blind_degrees / (max - min);
			slidersX[5] = percentage * (right - left) + left;
		}
			break;
		case 6: { //Turn Angle
			double max = 20;
			double min = 1;
			double percentage = turnAngle / (max - min);
			slidersX[6] = percentage * (right - left) + left;
		}
			break;
		case 7: { //Strength of Goal
			double max = 50;
			double min = 1;
			double percentage = endGoalStrength / (max - min);
			slidersX[7] = percentage * (right - left) + left;
		}
			break;
		}
	}
	slidersInitialized = true;
}

void inputRandomSeed() {
	string input;
	cout << "Input random seed: ";
	cin >> input;
	randomSeed = atoi(input.c_str());
}

constexpr unsigned int str2int(const char* str, int h = 0) {
	return !str[h] ? 5381 : (str2int(str, h + 1) * 33) ^ str[h];
}

void loadFile() {
	//get load file name and open
	string input;
	cout << "Load file name: ";
	cin >> input;
	ifstream inputData;
	inputData.open(input);

	//check if file was opened
	if (!inputData.is_open()) {
		cout << "Could not find file \"" << input << "\"\n";
		return;
	}

	//input the data
	int equalIndex = 0;
	char buffer[150];
	while (inputData >> buffer) {
		string str = buffer;
		equalIndex = str.find('=', 0);
		string parameterName = str.substr(0, equalIndex).c_str();
		switch (str2int(parameterName.c_str())) {
		case str2int("num"):
			BOID_NUMBER = atoi(str.substr(equalIndex + 1).c_str());
			break;
		case str2int("seed"):
			randomSeed = atoi(str.substr(equalIndex + 1).c_str());
			break;
		case str2int("rRepulsion"):
			radius_i = atof(str.substr(equalIndex + 1).c_str());
			break;
		case str2int("rOrientation"):
			radius_m = atof(str.substr(equalIndex + 1).c_str());
			break;
		case str2int("rAttraction"):
			radius_o = atof(str.substr(equalIndex + 1).c_str());
			break;
		case str2int("perception"):
			blind_degrees = atof(str.substr(equalIndex + 1).c_str());
			break;
		case str2int("turnRate"):
			turnAngle = atof(str.substr(equalIndex + 1).c_str());
			break;
		case str2int("timeStep"):
			step_size = atof(str.substr(equalIndex + 1).c_str());
			break;
		case str2int("goalStrength"):
			endGoalStrength = atof(str.substr(equalIndex + 1).c_str());
			break;
		case str2int("circleNumber"): {
			CIRCLE_OBSTACLE_NUMBER = atoi(str.substr(equalIndex + 1).c_str());
			for (int i = 0; i < CIRCLE_OBSTACLE_NUMBER; i++) {
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				circle_obstacles[i].center.x = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				circle_obstacles[i].center.y = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				circle_obstacles[i].size = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				circle_obstacles[i].repelStrength = atof(str.substr(equalIndex + 1).c_str());
			}
		}
			break;
		case str2int("lineNumber"): {
			LINE_OBSTACLE_NUMBER = atoi(str.substr(equalIndex + 1).c_str());
			for (int i = 0; i < LINE_OBSTACLE_NUMBER; i++) {
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				line_obstacles[i].start.x = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				line_obstacles[i].start.y = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				line_obstacles[i].end.x = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				line_obstacles[i].end.y = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				line_obstacles[i].length = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				line_obstacles[i].size = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				line_obstacles[i].repelStrength = atof(str.substr(equalIndex + 1).c_str());
				inputData >> buffer;
				str = buffer;
				equalIndex = str.find('=', 0);
				line_obstacles[i].start.nextDegrees = atof(str.substr(equalIndex + 1).c_str());
			}
		}
			break;
		}
	}

	//close file
	inputData.close();

	//restart program
	init();
	init2();
	slidersInitialized = false;
}

void saveFile() {
	//get save file name and open
	string input;
	cout << "Save file name: ";
	cin >> input;
	ofstream outputData;
	outputData.open(input);
	
	//output the data
	outputData << "num=" << BOID_NUMBER << endl;
	outputData << "seed=" << randomSeed << endl;
	outputData << "rRepulsion=" << radius_i << endl;
	outputData << "rOrientation=" << radius_m << endl;
	outputData << "rAttraction=" << radius_o << endl;
	outputData << "perception=" << blind_degrees << endl;
	outputData << "turnRate=" << turnAngle << endl;
	outputData << "timeStep=" << step_size << endl;
	outputData << "goalStrength=" << endGoalStrength << endl;

	//obstacles
	outputData << "circleNumber=" << CIRCLE_OBSTACLE_NUMBER << endl;
	for (int i = 0; i < CIRCLE_OBSTACLE_NUMBER; i++) {
		outputData << "circle" << i << "centerX=" << circle_obstacles[i].center.x << endl;
		outputData << "circle" << i << "centerY=" << circle_obstacles[i].center.y << endl;
		outputData << "circle" << i << "size=" << circle_obstacles[i].size << endl;
		outputData << "circle" << i << "repelStrength=" << circle_obstacles[i].repelStrength << endl;
	}
	outputData << "lineNumber=" << LINE_OBSTACLE_NUMBER << endl;
	for (int i = 0; i < LINE_OBSTACLE_NUMBER; i++) {
		outputData << "line" << i << "startX=" << line_obstacles[i].start.x << endl;
		outputData << "line" << i << "startY=" << line_obstacles[i].start.y << endl;
		outputData << "line" << i << "endX=" << line_obstacles[i].end.x << endl;
		outputData << "line" << i << "endY=" << line_obstacles[i].end.y << endl;
		outputData << "line" << i << "length=" << line_obstacles[i].length << endl;
		outputData << "line" << i << "size=" << line_obstacles[i].size << endl;
		outputData << "line" << i << "repelStrength=" << line_obstacles[i].repelStrength << endl;
		outputData << "line" << i << "startDegrees=" << line_obstacles[i].start.nextDegrees << endl;
	}

	//close file
	outputData.close();
}

static void mouse_active_move2(int x, int y) {
	double widthd = width2;
	double heightd = height2;
	double pointx = (x / widthd) * 2 - 1;
	double pointy = ((heightd - y) / heightd) * 2 - 1;
	double left = -0.25;
	double right = 0.7;

	if (movingSlider) {
		slidersX[sliderSelected] = pointx;
		if (slidersX[sliderSelected] < left) {
			slidersX[sliderSelected] = left;
		}
		else if (slidersX[sliderSelected] > right) {
			slidersX[sliderSelected] = right;
		}
		updateParameter();
	}

	display2();
}

static void mouse_press2(int button, int state, int x, int y) {
	double widthd = width2;
	double heightd = height2;
	double pointx = (x / widthd) * 2 - 1;
	double pointy = ((heightd - y) / heightd) * 2 - 1;
	double b_width = 0.3;
	double b_height = 0.15;

	if (button == 0 && state == 0) {
		//check if mouse on slider window
		double left = -0.25;
		double right = 0.7;
		double Height = 0.17;
		double error = 0.1;
		if (pointx > left - error && pointx < right + error) {
			for (int i = 0; i < 9; i++) {
				if ((pointy < 0.94 - Height * i) && (pointy > 0.88 - Height * i)) {
					if (i == 8) {
						inputRandomSeed();
						display2();
						return;
					}
					else {
						movingSlider = true;
						sliderSelected = i;
						display2();
						return;
					}
				}
			}
		}
		//check if clicking load or save
		else if (pointx > -0.5 - b_width && pointx < -0.5 + b_width && pointy > -0.8 - b_height && pointy < -0.8 + b_height) {
			load = true;
		}
		if (pointx > 0.5 - b_width && pointx < 0.5 + b_width && pointy > -0.8 - b_height && pointy < -0.8 + b_height) {
			save = true;
		}
	}

	if (button == 0 && state == 1) {
		if (movingSlider) {
			movingSlider = false;
		}
		//check if clicking load or save
		else if (pointx > -0.5 - b_width && pointx < -0.5 + b_width && pointy > -0.8 - b_height && pointy < -0.8 + b_height) {
			if (load) {
				loadFile();
			}
		}
		if (pointx > 0.5 - b_width && pointx < 0.5 + b_width && pointy > -0.8 - b_height && pointy < -0.8 + b_height) {
			if (save) {
				saveFile();
			}
		}
		load = false;
		save = false;
	}

	display2();
}

static void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//draw arena
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glColor3f(0.0f, 0.0f, 0.0f);
	glBegin(GL_POLYGON);
	int num_lines = 40;
	for (int i = 0; i < num_lines; i++) {
		double angle = i * 2 * 3.14159 / num_lines;
		glVertex2f(field_radius * cos(angle), field_radius * sin(angle));
	}
	glEnd();

	//draw end pin
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glColor3f(0.6f, 0.6f, 0.6f);
	glBegin(GL_POLYGON);
	for (int i = 0; i < num_lines; i++) {
		double angle = i * 2 * 3.14159 / num_lines;
		glVertex2f(endPoint.x + endPenRadius * cos(angle), endPoint.y + endPenRadius * sin(angle));
	}
	glEnd();

	//draw circle_obstacles
	for (int i = 0; i < CIRCLE_OBSTACLE_NUMBER; i++) {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin(GL_POLYGON);
		for (int j = 0; j < num_lines; j++) {
			double angle = j * 2 * 3.14159 / num_lines;
			glVertex2f(circle_obstacles[i].center.x + circle_obstacles[i].size / 3.0 * cos(angle), circle_obstacles[i].center.y + circle_obstacles[i].size / 3.0 * sin(angle));
		}
		glEnd();
	}

	//draw line obstacles
	for (int i = 0; i < LINE_OBSTACLE_NUMBER; i++) {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glColor3f(1.0f, 1.0f, 1.0f);
		glLineWidth(2);
		glBegin(GL_LINES);
		glVertex2f(line_obstacles[i].start.x, line_obstacles[i].start.y);
		glVertex2f(line_obstacles[i].end.x, line_obstacles[i].end.y);
		glEnd();
	}

	//draw boids
	for (int i = 0; i < BOID_NUMBER; i++) {
		//set rotation
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glPushMatrix;
		glTranslatef(boids[i].x, boids[i].y, 1);
		glRotatef(-90, 0, 0, 1);
		glRotatef(boids[i].degrees, 0, 0, 1);
		glTranslatef(-boids[i].x, -boids[i].y, -1);

		glBegin(GL_TRIANGLES);

		glColor3f(boids[i].c.red, boids[i].c.green, boids[i].c.blue);
		glVertex2f(boids[i].x, boids[i].y + triangle_size);
		glVertex2f(boids[i].x + .5 * triangle_size * sqrt(1.2), boids[i].y - .5 * triangle_size);
		glVertex2f(boids[i].x - .5 * triangle_size * sqrt(1.2), boids[i].y - .5 * triangle_size);
	
		glEnd();
		glPopMatrix;
	}

	//SwapBuffers
	glutSwapBuffers();

	glutSetWindow(windowTwo);

	if (callWindowOne) {
		callWindowOne = false;
	}
	else {
		display2();
	}
}

void print(double x, double y, char *string, void *font) {
	glRasterPos2d(x, y);
	glColor3f(1.0f, 1.0f, 1.0f);

	//get the length of the string to display
	int len = (int)strlen(string);

	//loop to display character by character
	for (int i = 0; i < len; i++) {
		glutBitmapCharacter(font, string[i]);
	}
}

static void display2() {
	if (!slidersInitialized) {
		initializeSliderPositions();
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	//print parameter names and values 
	double rowWidth = 0.17;
	char buffer[30];

	print(-0.95, 0.9, "Boid Number", GLUT_BITMAP_8_BY_13);
	sprintf_s(buffer, "%5d", BOID_NUMBER);
	print(0.8, 0.9, buffer, GLUT_BITMAP_8_BY_13);
	//
	print(-0.95, 0.9 - rowWidth * 1, "Radius Repulsion", GLUT_BITMAP_8_BY_13);
	sprintf_s(buffer, "%.3f", radius_i);
	print(0.8, 0.9 - rowWidth * 1, buffer, GLUT_BITMAP_8_BY_13);
	//
	print(-0.95, 0.9 - rowWidth * 2, "Radius Orientation", GLUT_BITMAP_8_BY_13);
	sprintf_s(buffer, "%.3f", radius_m);
	print(0.8, 0.9 - rowWidth * 2, buffer, GLUT_BITMAP_8_BY_13);
	//
	print(-0.95, 0.9 - rowWidth * 3, "Radius Attraction", GLUT_BITMAP_8_BY_13);
	sprintf_s(buffer, "%.3f", radius_o);
	print(0.8, 0.9 - rowWidth * 3, buffer, GLUT_BITMAP_8_BY_13);
	//
	print(-0.95, 0.9 - rowWidth * 4, "Time Step", GLUT_BITMAP_8_BY_13);
	sprintf_s(buffer, "%.3f", step_size);
	print(0.8, 0.9 - rowWidth * 4, buffer, GLUT_BITMAP_8_BY_13);
	//
	print(-0.95, 0.9 - rowWidth * 5, "Perception Angle", GLUT_BITMAP_8_BY_13);
	sprintf_s(buffer, "%5d", (int)blind_degrees);
	print(0.8, 0.9 - rowWidth * 5, buffer, GLUT_BITMAP_8_BY_13);
	//
	print(-0.95, 0.9 - rowWidth * 6, "Turn Angle", GLUT_BITMAP_8_BY_13);
	sprintf_s(buffer, "%5d", (int)turnAngle);
	print(0.8, 0.9 - rowWidth * 6, buffer, GLUT_BITMAP_8_BY_13);
	//
	print(-0.95, 0.9 - rowWidth * 7, "Strength of Goal", GLUT_BITMAP_8_BY_13);
	sprintf_s(buffer, "%.2f", endGoalStrength);
	print(0.8, 0.9 - rowWidth * 7, buffer, GLUT_BITMAP_8_BY_13);
	//
	print(-0.95, 0.9 - rowWidth * 8, "Random Seed", GLUT_BITMAP_8_BY_13);
	print(-0.25, 0.9 - rowWidth * 8, "<Click Here to input>", GLUT_BITMAP_8_BY_13);
	sprintf_s(buffer, "%27d", randomSeed);
	print(0.1f, 0.9 - rowWidth * 8, buffer, GLUT_BITMAP_8_BY_13);

	//print sliders
	double left = -0.25;
	double right = 0.7;
	double height = 0.17;
	for (int i = 0; i < 8; i++) {
		glColor3f(0.7f, 0.7f, 0.7f);
		glBegin(GL_POLYGON);
		glVertex2f(left, 0.92 - height * i);
		glVertex2f(left, 0.9 - height * i);
		glVertex2f(right, 0.9 - height * i);
		glVertex2f(right, 0.92 - height * i);
		glEnd();
	}

	//print slider positions
	double width = 0.02;
	for (int i = 0; i < 8; i++) {
		glColor3f(0.0f, 0.0f, 0.7f);
		glBegin(GL_POLYGON);
		glVertex2f(slidersX[i] - width, 0.94 - height * i);
		glVertex2f(slidersX[i] - width, 0.88 - height * i);
		glVertex2f(slidersX[i] + width, 0.88 - height * i);
		glVertex2f(slidersX[i] + width, 0.94 - height * i);
		glEnd();
	}

	//print load and save buttons
	glColor3f(0.7f, 0.7f, 1.0f);
	glBegin(GL_POLYGON);
	glVertex2f(-1.0, -1.0);
	glVertex2f(1.0, -1.0);
	glVertex2f(1.0, -0.6);
	glVertex2f(-1.0, -0.6);
	glEnd();

	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POLYGON);
	double b_width = 0.3;
	double b_height = 0.15;
	glVertex2f(-0.5 - b_width, -0.8 - b_height);
	glVertex2f(-0.5 + b_width, -0.8 - b_height);
	glVertex2f(-0.5 + b_width, -0.8 + b_height);
	glVertex2f(-0.5 - b_width, -0.8 + b_height);
	glEnd();

	glColor3f(0.0f, 0.4f, 0.0f);
	glBegin(GL_POLYGON);
	glVertex2f(0.5 - b_width, -0.8 - b_height);
	glVertex2f(0.5 + b_width, -0.8 - b_height);
	glVertex2f(0.5 + b_width, -0.8 + b_height);
	glVertex2f(0.5 - b_width, -0.8 + b_height);
	glEnd();

	glColor3f(1.0f, 1.0f, 1.0f);
	print(-0.6, -0.815, "Load", GLUT_BITMAP_TIMES_ROMAN_24);
	print(0.4, -0.815, "Save", GLUT_BITMAP_TIMES_ROMAN_24);

	//SwapBuffers
	glutSwapBuffers();

	glutSetWindow(windowOne);
	if (callWindowOne) {
		display();
	}
}

int compareX(const void *boid1, const void *boid2) {
	const Boid *b1 = (Boid *)boid1;
	const Boid *b2 = (Boid *)boid2;

	if (b1->x < b2->x) {
		return -1;
	}
	else {
		return 1;
	}
}

double distance(Boid boid1, Boid boid2) {
	double side1 = boid1.x - boid2.x;
	double side2 = boid1.y - boid2.y;
	return sqrt(side1 * side1 + side2 * side2);
}

double distanceFromLine(Boid boid, LineObstacle lineObstacle) {
	//get Line values
	double A, B, C;
	A = (lineObstacle.start.y - lineObstacle.end.y);
	B = (lineObstacle.start.x - lineObstacle.end.x);
	C = (lineObstacle.start.x * lineObstacle.end.y - lineObstacle.end.x * lineObstacle.start.y);

	//return Distance
	return abs(A * boid.x - B * boid.y + C) / sqrt(A * A + B * B);
}

double lineYAtBoid(Boid boid, LineObstacle lineObstacle) {
	double A, B, C;
	A = (lineObstacle.start.y - lineObstacle.end.y);
	B = (lineObstacle.end.x - lineObstacle.start.x);
	C = (lineObstacle.start.x * lineObstacle.end.y - lineObstacle.end.x * lineObstacle.start.y);

	return (-C - A * boid.x) / B;
}

bool affected(Boid boid, LineObstacle lineObstacle, double dist) {
	closestPointOnLine.nextDegrees = lineObstacle.start.nextDegrees;

	if (lineYAtBoid(boid, lineObstacle) < boid.y) {
		if (closestPointOnLine.nextDegrees > 90) {
			closestPointOnLine.nextDegrees += 90;
			if (closestPointOnLine.nextDegrees > 360) {
				closestPointOnLine.nextDegrees -= 360;
			}
			else if (closestPointOnLine.nextDegrees < 0) {
				closestPointOnLine.nextDegrees += 360;
			}
			closestPointOnLine.x = boid.x - sqrt(dist * dist / (1 + velocity(closestPointOnLine) * velocity(closestPointOnLine)));
			closestPointOnLine.y = boid.y + velocity(closestPointOnLine) * (closestPointOnLine.x - boid.x);
		}
		else {
			closestPointOnLine.nextDegrees -= 90;
			if (closestPointOnLine.nextDegrees > 360) {
				closestPointOnLine.nextDegrees -= 360;
			}
			else if (closestPointOnLine.nextDegrees < 0) {
				closestPointOnLine.nextDegrees += 360;
			}
			closestPointOnLine.x = boid.x + sqrt(dist * dist / (1 + velocity(closestPointOnLine) * velocity(closestPointOnLine)));
			closestPointOnLine.y = boid.y + velocity(closestPointOnLine) * (closestPointOnLine.x - boid.x);
		}
	}
	else {
		if (closestPointOnLine.nextDegrees < 90) {
			closestPointOnLine.nextDegrees -= 90;
			closestPointOnLine.nextDegrees += 180;
			if (closestPointOnLine.nextDegrees > 360) {
				closestPointOnLine.nextDegrees -= 360;
			}
			else if (closestPointOnLine.nextDegrees < 0) {
				closestPointOnLine.nextDegrees += 360;
			}
			closestPointOnLine.x = boid.x - sqrt(dist * dist / (1 + velocity(closestPointOnLine) * velocity(closestPointOnLine)));
			closestPointOnLine.y = boid.y + velocity(closestPointOnLine) * (closestPointOnLine.x - boid.x);
		}
		else {
			closestPointOnLine.nextDegrees -= 90;
			closestPointOnLine.nextDegrees += 180;
			if (closestPointOnLine.nextDegrees > 360) {
				closestPointOnLine.nextDegrees -= 360;
			}
			else if (closestPointOnLine.nextDegrees < 0) {
				closestPointOnLine.nextDegrees += 360;
			}
			closestPointOnLine.x = boid.x + sqrt(dist * dist / (1 + velocity(closestPointOnLine) * velocity(closestPointOnLine)));
			closestPointOnLine.y = boid.y + velocity(closestPointOnLine) * (closestPointOnLine.x - boid.x);
		}
	}

	if (closestPointOnLine.x < fmax(lineObstacle.start.x, lineObstacle.end.x) && closestPointOnLine.x > fmin(lineObstacle.start.x, lineObstacle.end.x)) {
		if (closestPointOnLine.y < fmax(lineObstacle.start.y, lineObstacle.end.y) && closestPointOnLine.y > fmin(lineObstacle.start.y, lineObstacle.end.y)) {
			return true;
		}
	}

	//check if start or end point is closest
	closestToEdgesDistance = distance(boid, lineObstacle.start);
	if (closestToEdgesDistance < lineObstacle.size) {
		closestPointOnLine.x = lineObstacle.start.x;
		closestPointOnLine.y = lineObstacle.start.y;
		closestToEdges = true;
		return true;
	}
	closestToEdgesDistance = distance(boid, lineObstacle.end);
	if (closestToEdgesDistance < lineObstacle.size) {
		closestPointOnLine.x = lineObstacle.end.x;
		closestPointOnLine.y = lineObstacle.end.y;
		closestToEdges = true;
		return true;
	}

	return false;
}

double slope(Boid boid1, Boid boid2) {
	return ((boid2.y - boid1.y) / (boid2.x - boid1.x));
}

double degreeAngle(Boid boid1, Boid boid2) {
	double line = slope(boid1, boid2);
	if (boid1.x > boid2.x) {
		return 180 + atan(line) * 180 / M_PI;
	}
	else if (boid1.y < boid2.y) {
		return atan(line) * 180 / M_PI;
	} 
	else {
		return 360 + atan(line) * 180 / M_PI;
	}
}

double velocity(Boid boid) {
	return tan(boid.nextDegrees / 180 * M_PI);
}

void updateBoidPositions() {
	//sort boids by x
	qsort(boids, BOID_NUMBER, sizeof(Boid), compareX);

	//check if boids in outer ring
	for (int i = 0; i < BOID_NUMBER; i++) {
		if (boids[i].inPen == true) {
			continue;
		}
		bool left = true;
		int j = i - 1;
		while (true) {
			if (j == BOID_NUMBER) {
				break;
			}
			else if (j < 0 || (abs(boids[i].x - boids[j].x) > radius_o && j < i)) {
				left = false;
				j = i;
			}
			else if (abs(boids[i].x - boids[j].x) > radius_o) {
				break;
			}
			if (i != j && abs(boids[i].x - boids[j].x) > radius_m) {
				if (boids[j].inPen == true) {
					//update j position
					if (left == true) {
						j--;
					}
					else {
						j++;
					}
					continue;
				}
				double dist = distance(boids[i], boids[j]);
				if (dist < radius_o && dist > radius_m) {
					double degrees;
					if (boids[i].degrees > 180) {
						degrees = boids[i].degrees - 180;
					}
					else {
						degrees = boids[i].degrees + 180;
					}
					if (abs(degreeAngle(boids[i], boids[j]) - degrees) < 360 - blind_degrees) {
						//update j position
						if (left == true) {
							j--;
						}
						else {
							j++;
						}
						continue;
					}

					//create degree rotation
					double adjustment = 1.0 - (dist - radius_m / (radius_o - radius_m));
					double ray = degreeAngle(boids[i], boids[j]);
					double angleTurn = abs(boids[i].nextDegrees - ray);

					double turnStrength = 40 - turnAngle;

					if (angleTurn < 180) {
						if (boids[i].nextDegrees < ray) {
							boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
						}
						else {
							boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
						}
					}
					else {
						angleTurn = 360 - angleTurn;
						if (boids[i].nextDegrees < ray) {
							boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
						}
						else {
							boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
						}
					}

					//make sure next degrees is not greater than 360 or less than 0
					if (boids[i].nextDegrees > 360) {
						boids[i].nextDegrees -= 360;
					}
					else if (boids[i].nextDegrees < 0) {
						boids[i].nextDegrees += 360;
					}
				}
			}
			//update j position
			if (left == true) {
				j--;
			}
			else {
				j++;
			}
		}
	}

	//check if boids in middle ring
	for (int i = 0; i < BOID_NUMBER; i++) {
		if (boids[i].inPen == true) {
			continue;
		}
		bool left = true;
		int j = i - 1;
		while (true) {
			if (j == BOID_NUMBER) {
				break;
			}
			else if (j < 0 || (abs(boids[i].x - boids[j].x) > radius_m && j < i)) {
				left = false;
				j = i;
			}
			else if (abs(boids[i].x - boids[j].x) > radius_m) {
				break;
			}
			if (i != j && abs(boids[i].x - boids[j].x) > radius_i) {
				if (boids[j].inPen == true) {
					//update j position
					if (left == true) {
						j--;
					}
					else {
						j++;
					}
					continue;
				}
				double dist = distance(boids[i], boids[j]);
				if (dist < radius_m && dist > radius_i) {
					double degrees;
					if (boids[i].degrees > 180) {
						degrees = boids[i].degrees - 180;
					}
					else {
						degrees = boids[i].degrees + 180;
					}
					if (abs(degreeAngle(boids[i], boids[j]) - degrees) < 360 - blind_degrees) {
						//update j position
						if (left == true) {
							j--;
						}
						else {
							j++;
						}
						continue;
					}

					//create degree rotation
					double adjustment = 1.0 - (dist - radius_i / (radius_m - radius_i));
					double ray = boids[j].degrees;
					double angleTurn = abs(boids[i].nextDegrees - ray);

					double turnStrength = 23 - turnAngle;

					if (angleTurn < 180) {
						if (boids[i].nextDegrees < ray) {
							boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
						}
						else {
							boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
						}
					}
					else {
						angleTurn = 360 - angleTurn;
						if (boids[i].nextDegrees < ray) {
							boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
						}
						else {
							boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
						}
					}

					//make sure next degrees is not greater than 360 or less than 0
					if (boids[i].nextDegrees > 360) {
						boids[i].nextDegrees -= 360;
					}
					else if (boids[i].nextDegrees < 0) {
						boids[i].nextDegrees += 360;
					}
				}
			}
			//update j position
			if (left == true) {
				j--;
			}
			else {
				j++;
			}
		}
	}

	//check if boids in inner ring
	bool resetBoid = false;
	for (int i = 0; i < BOID_NUMBER; i++) {
		if (boids[i].inPen == true) {
			continue;
		}
		bool left = true;
		int j = i - 1;
		while (true) {
			if (j == BOID_NUMBER) {
				break;
			}
			else if (j < 0 || (abs(boids[i].x - boids[j].x) > radius_i && j < i)) {
				left = false;
				j = i;
			}
			else if (abs(boids[i].x - boids[j].x) > radius_i) {
				break;
			}
			if (i != j) {
				if (boids[j].inPen == true) {
					//update j position
					if (left == true) {
						j--;
					}
					else {
						j++;
					}
					continue;
				}
				double dist = distance(boids[i], boids[j]);
				if (dist < radius_i) {
					double degrees;
					if (boids[i].degrees > 180) {
						degrees = boids[i].degrees - 180;
					}
					else {
						degrees = boids[i].degrees + 180;
					}
					if (abs(degreeAngle(boids[i], boids[j]) - degrees) < 360 - blind_degrees) {
						//update j position
						if (left == true) {
							j--;
						}
						else {
							j++;
						}
						continue;
					}

					//reset nextDegrees
					if (resetBoid == false) {
						boids[i].nextDegrees = boids[i].degrees;
						resetBoid = true;
					}

					//create degree rotation
					double adjustment = 1.0 - (dist / radius_i);
					double ray = degreeAngle(boids[i], boids[j]);
					if (ray < 180) {
						ray += 180;
					}
					else {
						ray -= 180;
					}
					double angleTurn = abs(boids[i].nextDegrees - ray);

					double turnStrength = 23 - turnAngle;

					if (angleTurn < 180) {
						if (boids[i].nextDegrees < ray) {
							boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
						}
						else {
							boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
						}
					}
					else {
						angleTurn = 360 - angleTurn;
						if (boids[i].nextDegrees < ray) {
							boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
						}
						else {
							boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
						}
					}

					//make sure next degrees is not greater than 360 or less than 0
					if (boids[i].nextDegrees > 360) {
						boids[i].nextDegrees -= 360;
					}
					else if (boids[i].nextDegrees < 0) {
						boids[i].nextDegrees += 360;
					}
				}
			}
			//update j position
			if (left == true) {
				j--;
			}
			else {
				j++;
			}
		}
		resetBoid = false;
	}

	//draw towards end point
	for (int i = 0; i < BOID_NUMBER; i++) {
		double ray = degreeAngle(boids[i], endPoint);
		double angleTurn = abs(boids[i].nextDegrees - ray);

		double turnStrength = 52 - endGoalStrength;
		if (boids[i].inPen == true) {
			turnStrength = -13 * (step_size / 0.06) + ((double(rand()) / double(RAND_MAX)) * (10) + 15);
		}

		if (angleTurn < 180) {
			if (boids[i].nextDegrees < ray) {
				boids[i].nextDegrees += (angleTurn / turnStrength);
			}
			else {
				boids[i].nextDegrees -= (angleTurn / turnStrength);
			}
		}
		else {
			angleTurn = 360 - angleTurn;
			if (boids[i].nextDegrees < ray) {
				boids[i].nextDegrees -= (angleTurn / turnStrength);
			}
			else {
				boids[i].nextDegrees += (angleTurn / turnStrength);
			}
		}

		//make sure next degrees is not greater than 360 or less than 0
		if (boids[i].nextDegrees > 360) {
			boids[i].nextDegrees -= 360;
		}
		else if (boids[i].nextDegrees < 0) {
			boids[i].nextDegrees += 360;
		}
	}

	//draw away from circle_obstacles
	penCount = 0;
	for (int i = 0; i < BOID_NUMBER; i++) {
		if (boids[i].inPen == true) {
			penCount++;
			continue;
		}
		for (int j = 0; j < CIRCLE_OBSTACLE_NUMBER; j++) {
			double dist = distance(boids[i], circle_obstacles[j].center);
			if (dist < circle_obstacles[j].size) {
				//create degree rotation
				double adjustment = 1.0 - (dist / circle_obstacles[j].size);
				double ray = degreeAngle(boids[i], circle_obstacles[j].center);
				if (ray < 180) {
					ray += 180;
				}
				else {
					ray -= 180;
				}
				double angleTurn = abs(boids[i].nextDegrees - ray);

				double turnStrength = circle_obstacles[j].repelStrength;

				if (angleTurn < 180) {
					if (boids[i].nextDegrees < ray) {
						boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
					}
					else {
						boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
					}
				}
				else {
					angleTurn = 360 - angleTurn;
					if (boids[i].nextDegrees < ray) {
						boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
					}
					else {
						boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
					}
				}

				//make sure next degrees is not greater than 360 or less than 0
				if (boids[i].nextDegrees > 360) {
					boids[i].nextDegrees -= 360;
				}
				else if (boids[i].nextDegrees < 0) {
					boids[i].nextDegrees += 360;
				}
			}
		}
	}

	//draw away from line_obstacles
	for (int i = 0; i < BOID_NUMBER; i++) {
		if (boids[i].inPen == true) {
			continue;
		}
		for (int j = 0; j < LINE_OBSTACLE_NUMBER; j++) {
			double dist = distanceFromLine(boids[i], line_obstacles[j]);
			if (dist < line_obstacles[j].size && affected(boids[i], line_obstacles[j], dist)) {
				//create degree rotation
				if (closestToEdges) {
					dist = closestToEdgesDistance;
					closestToEdges = false;
				}
				double adjustment = 1.0 - (dist / line_obstacles[j].size);
				double ray = degreeAngle(boids[i], closestPointOnLine);
				if (ray < 180) {
					ray += 180;
				}
				else {
					ray -= 180;
				}
				double angleTurn = abs(boids[i].nextDegrees - ray);

				double turnStrength = line_obstacles[j].repelStrength;

				if (angleTurn < 180) {
					if (boids[i].nextDegrees < ray) {
						boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
					}
					else {
						boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
					}
				}
				else {
					angleTurn = 360 - angleTurn;
					if (boids[i].nextDegrees < ray) {
						boids[i].nextDegrees -= (adjustment) * (angleTurn / turnStrength);
					}
					else {
						boids[i].nextDegrees += (adjustment) * (angleTurn / turnStrength);
					}
				}

				//make sure next degrees is not greater than 360 or less than 0
				if (boids[i].nextDegrees > 360) {
					boids[i].nextDegrees -= 360;
				}
				else if (boids[i].nextDegrees < 0) {
					boids[i].nextDegrees += 360;
				}
			}
		}
	}

	//advance boids path
	for (int i = 0; i < BOID_NUMBER; i++) {
		boids[i].nextX = boids[i].x + sqrt(step_size * step_size / (1 + velocity(boids[i]) * velocity(boids[i])));
		boids[i].nextY = boids[i].y + velocity(boids[i]) * (boids[i].nextX - boids[i].x);
		if (boids[i].degrees > 90 && boids[i].degrees < 270) {
			boids[i].nextY -= 2 * velocity(boids[i]) * (boids[i].nextX - boids[i].x);
			boids[i].nextX -= 2 * sqrt(step_size * step_size / (1 + velocity(boids[i]) * velocity(boids[i])));
		}
	}

	//assign new positions
	for (int i = 0; i < BOID_NUMBER; i++) {
		boids[i].x = boids[i].nextX;
		boids[i].y = boids[i].nextY;
		boids[i].degrees = boids[i].nextDegrees;
		if (distance(boids[i], endPoint) < endPenRadius) {
			boids[i].inPen = true;
		}
	}

	//adjust for leaving display area
	for (int i = 0; i < BOID_NUMBER; i++) {
		if (distance(boids[i], centerBoid) > field_radius) {
			//move distance 2 on slope towards center
			double v = slope(boids[i], centerBoid);
			if (boids[i].x < 0) {
				boids[i].x = boids[i].x + sqrt(field_radius * field_radius * 4 / (1 + v * v));
			}
			else {
				boids[i].x = boids[i].x - sqrt(field_radius * field_radius * 4 / (1 + v * v));
			}
			boids[i].y *= -1;
		}
	}
}

void createBoid(int position) {
	boids[position].x = ((double(rand()) / double(RAND_MAX)) * (1) - 4);
	boids[position].y = ((double(rand()) / double(RAND_MAX)) * (1) - 4);
	if (distance(boids[position], centerBoid) > field_radius) {
		//move distance 2 on slope towards center
		double v = slope(boids[position], centerBoid);
		if (boids[position].x < 0) {
			boids[position].x = boids[position].x + sqrt(field_radius * field_radius * 4 / (1 + v * v));
		}
		else {
			boids[position].x = boids[position].x - sqrt(field_radius * field_radius * 4 / (1 + v * v));
		}
		boids[position].y *= -1;
	}
	boids[position].c.red = ((double(rand()) / double(RAND_MAX)) * (1));
	boids[position].c.green = ((double(rand()) / double(RAND_MAX)) * (1));
	boids[position].c.blue = ((double(rand()) / double(RAND_MAX)) * (1));
	boids[position].inPen = false;

	double pert = ((double(rand()) / double(RAND_MAX)) * (36));
	boids[position].degrees = pert + 30;

	boids[position].nextDegrees = boids[position].degrees;
}

void idleFunc() {
	std::clock_t start = std::clock();

	//Rendering
	display();
	updateBoidPositions();

	while (((std::clock() - start) / (double)CLOCKS_PER_SEC) < refresh_rate) {
		;
	}

	if (penCount == BOID_NUMBER) {
		restartFrameCount++;
	}
	if (restartFrameCount == 50) {
		penCount = 0;
		restartFrameCount = 0;
		init();
	}

	if (run != true) {
		char c = '.';
		while ((c = _getch()) != ' ') {
			if (c == '\r') {
				run = true;
				break;
			}
		}
	}
}

void reshape(int w, int h) {
	width = w;
	height = h;

	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-field_radius, field_radius, -field_radius, field_radius, -field_radius, field_radius);
	glMatrixMode(GL_MODELVIEW);
}

void reshape2(int w, int h) {
	width2 = w;
	height2 = h;

	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	display2();
}

void init(void) {
	/* select clearing (background) color */
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float s = width / 2.0;
	if (height < width) s = height / 2;
	glOrtho(-field_radius, field_radius, -field_radius, field_radius, -field_radius, field_radius);
	glMatrixMode(GL_MODELVIEW);

	//create boids
	centerBoid.x = 0;
	centerBoid.y = 0;
	endPoint.x = 3.0;
	endPoint.y = 2.25;
	srand(randomSeed);
	for (int i = 0; i < BOID_NUMBER; i++) {
		createBoid(i);
	}
}

void init2(void) {
	/* select clearing (background) color */
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	glViewport(0, 0, width2, height2);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float s = width / 2.0;
	if (height2 < width2) s = height2 / 2;
	glOrtho(-1, 1, -1, 1, -1, 1);
	glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
	glutInit(&argc, argv);

	//interface window
	glutInitWindowSize(width2, height2);
	glutInitWindowPosition(399, 450);
	windowTwo = glutCreateWindow("Interface");
	init2();
	glutReshapeFunc(reshape2);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glutDisplayFunc(display2);
	glutMouseFunc(mouse_press2);
	glutMotionFunc(mouse_active_move2);

	//main window
	glutInitWindowSize(width, height);
	glutInitWindowPosition(900, 0);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	windowOne = glutCreateWindow("Boids");
	init();
	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutIdleFunc(idleFunc);

	//mouse callbacks
	createGLUTMenus();
	glutMouseFunc(mouse_press);
	glutMotionFunc(mouse_active_move);
	glutPassiveMotionFunc(mouse_passive_move);

	glutMainLoop();
	return 0;
}