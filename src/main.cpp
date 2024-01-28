#include <Arduino.h>
#include <DHT.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <U8g2lib.h>

#include "settings.h"

// set this to your actual display
U8G2_SSD1309_128X64_NONAME2_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/10,
                                             /* data=*/9, /* cs=*/12,
                                             /* dc=*/11, /* reset=*/13);

// sensor init
DHT dht(DHTPIN, DHTTYPE);

// encoder settings
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
int16_t last_encoder_pos = -1;
bool last_switch_state = HIGH;

// store temp and humidity
float temp = 0.0;
float humidity = 0.0;

enum class DisplayMode { SPLASH, STATUS, SETTINGS };
DisplayMode display = DisplayMode::SPLASH;

double setpoint, input, output;
PID heater_pid(&input, &output, &setpoint, 2, 5, 1, DIRECT);

void render_display() {
    u8g2.clearBuffer();  // wipe the display we always draw a fresh page

    switch (display) {
        case DisplayMode::SPLASH:
            // print text for now, replace with a graphic maybe?
            u8g2.setFont(u8g2_font_ncenB14_tr);
            u8g2.drawStr(0, 24, "SPLASH");
            break;
        case DisplayMode::STATUS:
            u8g2.setFont(u8g2_font_ncenB14_tr);
            u8g2.drawStr(0, 24, "STATUS");
            break;
        case DisplayMode::SETTINGS:
            u8g2.setFont(u8g2_font_ncenB14_tr);  // choose a suitable font
            u8g2.drawStr(0, 24, "SETTINGS");
            break;
    }

    u8g2.sendBuffer();  // send what we drew in memory to the display
}

void process_input() {
    // read the position, but divide it reduce the rate of change
    int16_t encoder_pos = encoder.read() / 4;
    bool switch_state =
        digitalRead(ENCODER_PIN_SWITCH);  // Read the switch state

    if (encoder_pos != last_encoder_pos) {
        if (encoder_pos > last_encoder_pos) {
            // the encoder was turned clockwise
            // change some state here (navigate menu)

        } else {
            // the encoder was turned counterclockwise
            // more changes to state (navigate menu)
        }
        last_encoder_pos = encoder_pos;
    }

    if (switch_state != last_switch_state && switch_state == LOW) {
        // the switch was pressed
        // do something when the switch is pressed
    }
    last_switch_state = switch_state;

    // read from DHT sensor
    humidity = dht.readHumidity();
    temp = dht.readTemperature();
}

void heat() {
    // update the PID controller
    input = temp;
    heater_pid.Compute();

    // control the relay
    if (output > 127) {
        digitalWrite(RELAY_PIN, HIGH);
    } else {
        digitalWrite(RELAY_PIN, LOW);
    }
}

void setup() {
    // setup the screen and sensor
    u8g2.begin();
    dht.begin();
    // setup the relay
    pinMode(RELAY_PIN, OUTPUT);

    // initialize the PID controller
    setpoint = 25;  // Change this to your desired temperature
    heater_pid.SetMode(AUTOMATIC);
    heater_pid.SetOutputLimits(0, 255);  // Change this if needed

    // render the splash page for a second
    render_display();
    delay(1000);
    // change to STATUS page
    display = DisplayMode::STATUS;
}

void loop() {
    // put your main code here, to run repeatedly:
    render_display();
    process_input();
}