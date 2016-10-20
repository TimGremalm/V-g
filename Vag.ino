#include <math.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
	#include <avr/power.h>
#endif
//#include <PrintEx.h>		//https://github.com/Chris--A/PrintEx

#define WS2812PIN 2
#define WS2812PIXELS 144
#define WS2812OFFSETANGLE 300

//StreamEx serialf = Serial;


// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//	 NEO_KHZ800	800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//	 NEO_KHZ400	400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//	 NEO_GRB		 Pixels are wired for GRB bitstream (most NeoPixel products)
//	 NEO_RGB		 Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(WS2812PIXELS, WS2812PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.	Avoid connecting
// on a live circuit...if you must, connect GND first.

int calLoadCellA = 0;
int calLoadCellB = 0;
int calLoadCellC = 0;
int SmoothedAngleOut = 0;
int Force = 0;
int Angle = 0;

/** Helper method that converts hue to rgb */
float hueToRgb(float p, float q, float t) {
	if (t < (float)0)
			t += (float)1;
	if (t > (float)1)
			t -= (float)1;
	if (t < (float)1/(float)6)
			return p + (q - p) * (float)6 * t;
	if (t < (float)1/(float)2)
			return q;
	if (t < (float)2/(float)3)
			return p + (q - p) * ((float)2.0/ (float)3.0 - t) * (float)6;
	return p;
}

/* Converts an HSL color value to RGB. Conversion formula
-- adapted from http://en.wikipedia.org/wiki/HSL_color_space.
-- Assumes h, s, and l are contained in the set [0, 1] and
-- returns r, g, and b in the set [0, 1].
--
-- @param	 Number	h			 The hue
-- @param	 Number	s			 The saturation
-- @param	 Number	l			 The lightness
-- @return	Array					 The RGB representation*/
void hslToRgb(float h, float s, float l, int rgb[]) {
	float r, g, b;

	if (s == (float)0) {
			r = g = b = l; // achromatic
	} else {
			float q = l < 0.5 ? l * (1 + s) : l + s - l * s;
			float p = 2 * l - q;
			r = hueToRgb(p, q, h + (float)1/(float)3);
			g = hueToRgb(p, q, h);
			b = hueToRgb(p, q, h - (float)1/(float)3);
	}
	rgb[0] = (int) (r * 255);
	rgb[1] = (int) (g * 255);
	rgb[2] = (int) (b * 255);
}

uint32_t HSLtoColor(float H, float S, float L) {
	int nRGB[3] = {0,0,0};
	hslToRgb(H, S, L, nRGB);
	return strip.Color(nRGB[0], nRGB[1], nRGB[2]);
}


#define radian2degree(a) (a * 57.295779513082)
#define degree2radian(a) (a * 0.017453292519)

float calcTotalResultantAngle(float F1Newton, float F2Newton, float F3Newton, float F1Angle, float F2Angle, float F3Angle) {
	float Force1 = F1Newton;
	float Force2 = F2Newton;
	float Force3 = F3Newton;
	float Force1Angle = degree2radian(F1Angle);
	float Force2Angle = degree2radian(F2Angle);
	float Force3Angle = degree2radian(F3Angle);

	float ForceResultX = (cos(Force1Angle) * Force1) + (cos(Force2Angle) * Force2) + (cos(Force3Angle) * Force3);
	float ForceResultY = (sin(Force1Angle) * Force1) + (sin(Force2Angle) * Force2) + (sin(Force3Angle) * Force3);

	float ForceResultOverall = sqrt(pow(ForceResultX, 2) + pow(ForceResultY, 2));
	float Degreee = radian2degree(asin(ForceResultY / ForceResultOverall));

	float DegreeOut = 0;
	if (ForceResultX >= 0 && ForceResultY >= 0) {
		//Quadrant 1
		DegreeOut = Degreee;
	}
	if (ForceResultX < 0 && ForceResultY >= 0) {
		//Quadrant 2
		DegreeOut = 180 - Degreee;
	}
	if (ForceResultX < 0 && ForceResultY < 0) {
		//Quadrant 3
		DegreeOut = 180 + (Degreee * -1);
	}
	if (ForceResultX >= 0 && ForceResultY < 0) {
		//Quadrant 4
		DegreeOut = 360 + Degreee;
	}

	return DegreeOut;
}

float calcTotalResultantForce(float F1Newton, float F2Newton, float F3Newton, float F1Angle, float F2Angle, float F3Angle) {
	float Force1 = F1Newton;
	float Force2 = F2Newton;
	float Force3 = F3Newton;
	float Force1Angle = degree2radian(F1Angle);
	float Force2Angle = degree2radian(F2Angle);
	float Force3Angle = degree2radian(F3Angle);

	float ForceResultX = (cos(Force1Angle) * Force1) + (cos(Force2Angle) * Force2) + (cos(Force3Angle) * Force3);
	float ForceResultY = (sin(Force1Angle) * Force1) + (sin(Force2Angle) * Force2) + (sin(Force3Angle) * Force3);

	return sqrt(pow(ForceResultX, 2) + pow(ForceResultY, 2));
}

void CalcAngleAndForce() {
	int samples = 1;
	int Forces[samples];
	int Angles[samples];
	//Serial.println("CalcAngleAndForce");
	for(int i = 0; i <= samples; i += 1) {
		int loadCellA = analogRead(A4);
		int loadCellB = analogRead(A5);
		int loadCellC = analogRead(A6);

		loadCellA -= calLoadCellA;
		loadCellB -= calLoadCellB;
		loadCellC -= calLoadCellC;

		float resultantForce = calcTotalResultantForce(loadCellA, loadCellB, loadCellC, 0, 120, 240);
		Forces[i] = (int) resultantForce;

		float resultantAngle = calcTotalResultantAngle(loadCellA, loadCellB, loadCellC, 0, 120, 240);
		Angles[i] = ((int) resultantAngle) % 360;

		//delay(1);
		//Serial.println(i);
	}

	float ForceSum = 0;
	float AngleSum = 0;
	for(int i = 0; i <= samples; i += 1) {
		ForceSum += (float)Forces[i];
		AngleSum += (float)Angles[i];
	}

	ForceSum = ForceSum / (float)samples;
	AngleSum = AngleSum / (float)samples;

	Force = (int) ForceSum;
	Angle = (int) AngleSum;
	Angle += 360;
	Angle = Angle % 360;
}

void SmoothAngle(int AngleIn) {
	int diffToZero = 0 - SmoothedAngleOut;
	int goalAfterDiffToZero = AngleIn + diffToZero;
	goalAfterDiffToZero += 360;
	goalAfterDiffToZero = goalAfterDiffToZero % 360;
	int posNegToAdd;
	if (goalAfterDiffToZero > 180) {
		posNegToAdd = 1;
	} else {
		posNegToAdd = -1;
	}

	//=MOD((0-J2)*K2, 360)*K2
	int shortestPath = (0-goalAfterDiffToZero) * posNegToAdd;
	shortestPath += 360;
	shortestPath = shortestPath % 360;
	shortestPath = shortestPath * posNegToAdd;
	shortestPath = constrain(shortestPath, -10, 10);

	SmoothedAngleOut -= shortestPath;
	SmoothedAngleOut = SmoothedAngleOut % 360;
}

void DrawBalance(int Angle, int Force) {
	//Calculate red green ratio
	float ForceRatio = (float)Force / 1023;
	float ForceRatioRed = constrain(ForceRatio, 0.1, 0.6);
	float ForceRatioGreen = constrain((float)1-ForceRatio, 0.1, 1);

	int ForceAngleRed = 180 * ForceRatioRed;
	int ForceAngleGreen = 180 * ForceRatioGreen;
	//serialf.println(ForceAngleRed);
	//serialf.println(ForceAngleGreen);

	//Calculate angle
	Angle += WS2812OFFSETANGLE;
	Angle = Angle % 360;
	Angle = 360 - Angle;	//Invert it
	Angle = Angle % 360;

	float LedsDegreeRatio = ((float) WS2812PIXELS) / 360; //0.28
	float DegreeLedsRatio = 360 / ((float) WS2812PIXELS); //3.6

	//Make all segments black
	for(int i = 0; i <= WS2812PIXELS; i += 1) {
		uint32_t color = strip.Color(0, 0, 0);
		strip.setPixelColor(i, color);
	}

	//Loop through pixel step in the red pointer
	int AngleRange = ForceAngleRed;
	for(float i = Angle; i <= Angle+AngleRange; i += floor(DegreeLedsRatio)) {
		float iProgress = i-Angle;
		float iProgressRatio = iProgress / AngleRange;
		float iL = (1-iProgressRatio)/2;
		iL = iL*iL;

		uint32_t color = HSLtoColor(0, 1, iL);

		int iPos = floor(i * LedsDegreeRatio);
		iPos = iPos % WS2812PIXELS;

		int iNeg = floor((Angle-iProgress) * LedsDegreeRatio);
		iNeg += WS2812PIXELS;
		iNeg = iNeg % WS2812PIXELS;

		strip.setPixelColor(iPos, color);
		strip.setPixelColor(iNeg, color);
	}
	Serial.print("\r\n");

	//Loop through pixel step in the green pointer
	Angle = (Angle+180);	//Phaseshift green indicator by 180 degrees
	Angle = Angle % 360;
	AngleRange = ForceAngleGreen;
	for(float i = Angle; i <= Angle+AngleRange; i += floor(DegreeLedsRatio)) {
		float iProgress = i-Angle;
		float iProgressRatio = iProgress / AngleRange;
		float iL = (1-iProgressRatio)/2;
		iL = iL * iL;

		uint32_t color = HSLtoColor(0.3, 1, iL);

		int iPos = floor(i * LedsDegreeRatio);
		iPos = iPos % WS2812PIXELS;

		int iNeg = floor((Angle-iProgress) * LedsDegreeRatio);
		iNeg += WS2812PIXELS;
		iNeg = iNeg % WS2812PIXELS;

		strip.setPixelColor(iPos, color);
		strip.setPixelColor(iNeg, color);
	}
	strip.show();
}

void setup() {
	// This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
	#if defined (__AVR_ATtiny85__)
		if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
	#endif
	// End of trinket special code

	strip.begin();
	strip.show(); // Initialize all pixels to 'off'

	Serial.begin(9600);

	calLoadCellA = analogRead(A4);
	calLoadCellB = analogRead(A5);
	calLoadCellC = analogRead(A6);
}

void loop() {
	CalcAngleAndForce();

	//serialf.printf("Angle Force: %d %d\n", Angle, Force);
	Serial.print("Angle Force: ");
	Serial.print(Angle);
	Serial.print(" ");
	Serial.print(Force);
	Serial.print("\r\n");

	//SmoothedAngleOut = 230;
	//Angle = 30;

	SmoothAngle(Angle);
	DrawBalance(SmoothedAngleOut, Force);

	delay(5);
}

