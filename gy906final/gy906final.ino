//June 20
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

int i = 0;


void setup()
{
  Serial.begin(9600);
  while (!Serial);

  //Serial.println("Adafruit MLX90614 test");

  if (!mlx.begin())
  {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

}

void loop()
{
  //Serial.print("Object = "); Serial.print(mlx.readObjectTempC() - 11); Serial.println("*C");
  i = mlx.readObjectTempC() - 11;
  
  if (i < 38)
  {
    Serial.print("Normal Temp");
  }
  else
  {
    Serial.print("High Temp");
  }


  delay(500);
  Serial.println();
  
}
