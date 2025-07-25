#include <Wire.h>
#include <MPU9250_asukiaaa.h>

// Pines I2C 
#define SDA_PIN 10
#define SCL_PIN 8

MPU9250_asukiaaa mpu;

// Variables para el filtro complementario
float roll, pitch, yaw;
float alpha = 0.9; // Factor del filtro complementario (0.9 = 90% giroscopio, 10% acelerómetro)
float dt; // Intervalo de tiempo entre lecturas
unsigned long lastTime;

// Offsets del giroscopio 
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Inicializa I2C con pines personalizados

  // Inicializa el MPU
  mpu.setWire(&Wire);
  uint8_t sensorId;
  if (mpu.readId(&sensorId) != 0) {
    Serial.println("No se pudo inicializar el MPU9250. Verifica conexiones.");
    while (1);
  }

  mpu.beginAccel();
  mpu.beginGyro();

  if (mpu.accelUpdate() != 0 || mpu.gyroUpdate() != 0) {
    Serial.println("No se pudo leer datos del MPU9250. Verifica conexiones.");
    while (1);
  }

  Serial.println("MPU9250 inicializado!");

  // Calibrar offsets del giroscopio
   calibrateGyro();
  
  lastTime = micros();
}

void loop() {
  // Obtener datos del sensor
  mpu.accelUpdate();
  mpu.gyroUpdate();

  // Calcular dt (en segundos)
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Obtener datos crudos
  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();
  float gx = mpu.gyroX() - gyroXoffset;
  float gy = mpu.gyroY() - gyroYoffset;
  float gz = mpu.gyroZ() - gyroZoffset;

  // Calcular ángulos con el acelerómetro
  float accelRoll = atan2(ay, az) * 180 / PI;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // Integrar giroscopio para obtener ángulos
  roll += gx * dt;
  pitch += gy * dt;
  yaw += gz * dt;

  // Aplicar filtro complementario
  roll = alpha * roll + (1 - alpha) * accelRoll;
  pitch = alpha * pitch + (1 - alpha) * accelPitch;
  // Nota: yaw no se corrige con acelerómetro, solo con giroscopio (puede derivar)

  // Imprimir resultados
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print("\tPitch: "); Serial.print(pitch);
  Serial.print("\tYaw: "); Serial.print(yaw);
  Serial.print("\tgx: "); Serial.print(gx);
  Serial.print("\tgy: "); Serial.print(gy);
  Serial.print("\tgz: "); Serial.print(gz);
  Serial.println();

  delay(10); //  frecuencia 
}

// Función para calibrar offsets del giroscopio (opcional)
void calibrateGyro() {
  const int numSamples = 3000;
  float sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("Calibrando giroscopio... Mantén el sensor quieto.");
  for (int i = 0; i < numSamples; i++) {
    mpu.gyroUpdate();
    sumX += mpu.gyroX();
    sumY += mpu.gyroY();
    sumZ += mpu.gyroZ();
    delay(3);
  }

  gyroXoffset = sumX / numSamples;
  gyroYoffset = sumY / numSamples;
  gyroZoffset = sumZ / numSamples;

  Serial.println("Calibración completa.");
}