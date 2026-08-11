# ESP32-S3_Drum_Midi_Controller
Convert piezo trigger inputs to Midi percussion control messages over USB<br><br>

Arduino IDE Tools menu settings to configure after selecting board type ESP32S3 Dev Module:

| Menu Option     | Required Setting        | Why it matters                                                                                              |
|-----------------|-------------------------|-------------------------------------------------------------------------------------------------------------|
| USB Mode        | "USB-OTG (TinyUSB)"     | Forces the ESP32-S3 to use the Adafruit TinyUSB stack to act as a class-compliant MIDI device.              |
| USB CDC On Boot | "Enabled"               | Tells the chip to route standard Serial.print() communications over the same USB connection alongside MIDI. |
| Upload Mode     | "USB-OTG CDC (TinyUSB)" | Allows the Arduino IDE to auto-reset the board and flash code directly over the native USB port.            |

<br><br>![PCB](/stuff/Trigger_Flowchart.png)<BR><BR>
