const void(*bootloader)() = ((const void(*)())0x1DE0);
const int BL_PIN = 4;

int led = 0;
void setup()
{
    pinMode(BL_PIN, INPUT);
    digitalWrite(BL_PIN, HIGH);
    if( digitalRead(BL_PIN) == LOW ) // or HIGH, depending on which state you want the bootloader to run
        bootloader();
    pinMode(led, OUTPUT);
}


void loop()                     
{
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);
}
