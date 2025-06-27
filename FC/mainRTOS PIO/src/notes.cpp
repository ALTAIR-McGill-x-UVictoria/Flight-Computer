#include "notes.h"
#include "Arduino.h"

void playStartupJingle() {
  // Short pause
  delay(200);
  
  // Quick confirmation beeps
  tone(BUZZER_PIN, NOTE_E4, 100);
  delay(150);
  tone(BUZZER_PIN, NOTE_G4, 100);
  delay(150);
  tone(BUZZER_PIN, NOTE_G5, 200);
  delay(300);
}

void playWaitingJingle(){
    tone(BUZZER_PIN, NOTE_E4, 100);
    delay(200);
    tone(BUZZER_PIN, NOTE_E4, 100);
    delay(200);
    tone(BUZZER_PIN, NOTE_E4, 100);
    delay(200);
    tone(BUZZER_PIN, NOTE_E4, 100);

    
}

void playErrorJingle() {

  while(1){
    tone(BUZZER_PIN, NOTE_G4, 100);
    delay(150);
    tone(BUZZER_PIN, NOTE_CS5, 100);
    delay(150);
    tone(BUZZER_PIN, NOTE_G4, 100);
    
    delay(3000);
  }
}

void playPingJingle() {
  tone(BUZZER_PIN, NOTE_C5, 100);
  delay(150);
  tone(BUZZER_PIN, NOTE_E5, 100);
  delay(150);
  tone(BUZZER_PIN, NOTE_C5, 100);
  delay(150);
  tone(BUZZER_PIN, NOTE_E5, 100);
}