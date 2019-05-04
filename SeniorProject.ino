const int trackEncoderA = 2;
const int trackEncoderB = 3;
volatile long trackEncoder = 0;

void setup() {
  Serial.begin(9600);
  pinMode(trackEncoderA, INPUT);
  pinMode(trackEncoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(trackEncoderA), trackEncoderPinChangeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(trackEncoderB), trackEncoderPinChangeB, CHANGE);

  Serial.println("Hello");
}

void loop() {


}

void trackEncoderPinChangeA(){
  trackEncoder += digitalRead(trackEncoderA) == digitalRead(trackEncoderB) ? -1:1;
  Serial.println(trackEncoder);
}

void trackEncoderPinChangeB(){
  trackEncoder += digitalRead(trackEncoderA) != digitalRead(trackEncoderB) ? -1:1;
}
