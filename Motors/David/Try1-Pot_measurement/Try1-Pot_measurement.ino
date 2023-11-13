                    // Seen from the back limiter:
const int pos1 = 4;      // L limit: 3.5 cm, R: 0 cm
const int pos2 = 0;      // R & L: 1 cm



void setup() {
  // put your setup code here, to run once:
  pinMode(pos1,INPUT);
  pinMode(pos2,INPUT);
  Serial.begin(115200);
  while(!Serial);
}

void loop() {
  Serial.print("Pot1:");
  Serial.print(analogRead(pos1));
  Serial.print("\tPot2:");
  Serial.println(analogRead(pos2)/1024.0*);

  delay(500);
}
