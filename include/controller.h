#define speakerPin 30
#include "pitches.h"
#include <Arduino.h>

void red_alert();
void play_coin();
void play_one_up();
void play_fireball();


void play_coin() {
  tone(speakerPin, NOTE_B5, 100);
  delay(100);
  tone(speakerPin, NOTE_E6, 850);
  delay(800);
  noTone(speakerPin);
}

void play_one_up() {
  tone(speakerPin, NOTE_E6, 125);
  delay(130);
  tone(speakerPin, NOTE_G6, 125);
  delay(130);
  tone(speakerPin, NOTE_E7, 125);
  delay(130);
  tone(speakerPin, NOTE_C7, 125);
  delay(130);
  tone(speakerPin, NOTE_D7, 125);
  delay(130);
  tone(speakerPin, NOTE_G7, 125);
  delay(125);
  noTone(speakerPin);
}

void play_fireball()   {
  tone(speakerPin, NOTE_G4, 35);
  delay(35);
  tone(speakerPin, NOTE_G5, 35);
  delay(35);
  tone(speakerPin, NOTE_G6, 35);
  delay(35);
  noTone(speakerPin);
}

void red_alert() {
  Serial.println("Red Alert!");
  for (int i = 500; i < 750; i++) {
    tone(speakerPin, i);
    delay(3);
  }
  noTone(speakerPin);
  delay (1000);
}