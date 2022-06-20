const int xInput = 13;
const int yInput = 14;
const int zInput = 15;

// initialize minimum and maximum Raw Ranges for each axis
int RawMin = 0;
int RawMax = 1023;

// Take multiple samples to reduce noise
const int sampleSize = 10;

void setup()
{
  //analogReference(EXTERNAL);
  Serial.begin(115200);
}

void loop()
{
  //Read raw values
  int xRaw = ReadAxis(xInput);
  int yRaw = ReadAxis(yInput);
  int zRaw = ReadAxis(zInput);

  // Convert raw values to 'milli-Gs"
  long xScaled = map(xRaw, RawMin, RawMax, -3000, 3000);
  long yScaled = map(yRaw, RawMin, RawMax, -3000, 3000);
  long zScaled = map(zRaw, RawMin, RawMax, -3000, 3000);

  // re-scale to fractional Gs
  float xAccel = xScaled / 1000.0;
  float yAccel = yScaled / 1000.0;
  float zAccel = zScaled / 1000.0;

  Serial.print("X, Y, Z  :: ");
  //Serial.print(xRaw);
  //Serial.print(", ");
  //Serial.print(yRaw);
  //Serial.print(", ");
  //Serial.print(zRaw);
  //Serial.print(" :: ");
  Serial.print(xAccel, 0);
  Serial.print("G, ");
  Serial.print(yAccel, 0);
  Serial.print("G, ");
  Serial.print(zAccel, 0);
  Serial.println("G");
  //Serial.print('\n');


  delay(1000);
}



// Take samples and return the average
int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading / sampleSize;
}
