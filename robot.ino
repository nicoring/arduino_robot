#include "kmeans.c"

/*  pins for motor 1 */
int enablePin_1 = 5;
int in1Pin_1 = 4;
int in2Pin_1 = 3;

/* pins for motor 2*/
int enablePin_2 = 6;
int in3Pin_2 = 8;
int in4Pin_2 = 7;

int DEFAULT_SPEED = 255;
int STOP_TIME = 200; //ms

/* Photodiod pins */
int diod1Pin = A4;
int diod2Pin = A5;
int diod3Pin = A6;	
int diod4Pin = A7;

bool collecting = true;
int collectedPoints = 0;
int numberOfCollections = 500;
int numDataPoints = numberOfCollections * 4;
double** dataPoints;

enum Direction {FORWARD, BACKWARD};

int aprintf(char *str, ...) {
	int i, j, count = 0;
 
	va_list argv;
	va_start(argv, str);
	for(i = 0, j = 0; str[i] != '\0'; i++) {
		if (str[i] == '%') {
			count++;
 
			Serial.write(reinterpret_cast<const uint8_t*>(str+j), i-j);
 
			switch (str[++i]) {
				case 'd': Serial.print(va_arg(argv, int));
					break;
				case 'l': Serial.print(va_arg(argv, long));
					break;
				case 'f': Serial.print(va_arg(argv, double));
					break;
				case 'c': Serial.print((char) va_arg(argv, int));
					break;
				case 's': Serial.print(va_arg(argv, char *));
					break;
				case '%': Serial.print("%");
					break;
				default:;
			};
 
			j = i+1;
		}
	};
	va_end(argv);
 
	if(i > j) {
		Serial.write(reinterpret_cast<const uint8_t*>(str+j), i-j);
	}
 
	return count;
}

void setup() {
	Serial.begin(9600);

	pinMode(enablePin_1, OUTPUT);
	pinMode(in1Pin_1, OUTPUT);
	pinMode(in2Pin_1, OUTPUT);
	pinMode(enablePin_2, OUTPUT);
	pinMode(in3Pin_2, OUTPUT);
	pinMode(in4Pin_2, OUTPUT);

	digitalWrite(diod1Pin, HIGH);
	digitalWrite(diod2Pin, HIGH);
	digitalWrite(diod3Pin, HIGH);
	digitalWrite(diod4Pin, HIGH);

	dataPoints = (double**)calloc(numDataPoints, sizeof(double*));

	Serial.println("Car initialized, ready to go!");
}

void loop() {
	Serial.println("loop");
	if (collecting && collectedPoints <= numberOfCollections) {
		/*driveRandom();*/
		collectIntialData();
		collectedPoints++;
		delay(100);
	} else if (collecting) {
		computeMeans();
		collecting = false;
	} else {
		mainLoop();
	}

}

void mainLoop() {
	// RotateLeft(1000);
	// RotateRight(1000);
	// RotateLeft(1000);
	// RotateRight(1000);
	// Stop();
	// delay(2000);
	delay(2000);
}

void Forward(int time)
{
  Serial.println("move forward");
	setMotors(DEFAULT_SPEED, FORWARD);
	delay(time);
	Stop();
	delay(STOP_TIME);
  /*digitalWrite(motor1PinA, HIGH);
  digitalWrite(motor1PinB, LOW);
  digitalWrite(motor2PinA, HIGH);
  digitalWrite(motor2PinB, LOW);*/
}

void Backward(int time) // inverse voltage of both motors
{
  Serial.println("move backward");
	setMotors(DEFAULT_SPEED, BACKWARD);
	delay(time);
	Stop();
	delay(STOP_TIME);
  /*digitalWrite(motor1PinA, LOW);
  digitalWrite(motor1PinB, HIGH);
  digitalWrite(motor2PinA, LOW);
  digitalWrite(motor2PinB, HIGH);*/
}

void RotateLeft(int time) // inverse voltage of only one motor
{
  Serial.println("rotate left");
	setMotorLeft(DEFAULT_SPEED, BACKWARD);
	setMotorRight(DEFAULT_SPEED, FORWARD);
	delay(time);
	Stop();
	delay(STOP_TIME);
  /*digitalWrite(motor1PinA, LOW);
  digitalWrite(motor1PinB, HIGH);
  digitalWrite(motor2PinA, HIGH);
  digitalWrite(motor2PinB, LOW);*/
}

void RotateRight(int time) // inverse voltage of only one motor
{
  Serial.println("rotate right");
	setMotorLeft(DEFAULT_SPEED, FORWARD);
	setMotorRight(DEFAULT_SPEED, BACKWARD);
	delay(time);
	Stop();
	delay(STOP_TIME);
}

void Stop()
{
  Serial.println("stop");
  digitalWrite(in1Pin_1, LOW);
  digitalWrite(in2Pin_1, LOW);
  digitalWrite(in3Pin_2, LOW);
  digitalWrite(in4Pin_2, LOW);
  analogWrite(enablePin_1, 0);
  analogWrite(enablePin_2, 0);
}

void setMotors(int speed, int direction) {
	setMotorLeft(speed, direction);
	setMotorRight(speed, direction);
}

void setMotorLeft(int speed, int direction) {
	int controlBit = direction == FORWARD ? 1 : 0;
  analogWrite(enablePin_1, speed);
  digitalWrite(in1Pin_1, controlBit);
  digitalWrite(in2Pin_1, ! controlBit);
}

void setMotorRight(int speed, int direction) {
	int controlBit = direction == FORWARD ? 1 : 0;
  analogWrite(enablePin_2, speed);
  digitalWrite(in3Pin_2, ! controlBit);
  digitalWrite(in4Pin_2, controlBit);
}

void computeMeans() {
	int dimension = 1;
	int clusters = 3;
	double errorTolerance = 0.01;
	double** centroids = (double**)calloc(clusters, sizeof(double*));

	int* labels = k_means(dataPoints, numDataPoints, dimension, clusters, errorTolerance, centroids);
	for (int i = 0; i < numDataPoints; i++) {
		aprintf("data point %d is in cluster %d\n", dataPoints[i][0]	, labels[i]);
	}
	free(labels);
	free(centroids);
}

void collectIntialData() {
	int* values = getSensorValues();
	dataPoints[collectedPoints][0] = (double) values[0];
	dataPoints[collectedPoints + 1][0] = (double) values[1];
	dataPoints[collectedPoints + 2][0] = (double) values[2];
	dataPoints[collectedPoints + 3][0] = (double) values[3];
}

int* getSensorValues() {
	int values[4];
	values[0] = analogRead(diod1Pin);
	values[1] = analogRead(diod2Pin);
	values[2] = analogRead(diod3Pin);
	values[3] = analogRead(diod4Pin);
	Serial.println("" + values[0] + values[1] + values[2] + values[3]);
	return values;
}
