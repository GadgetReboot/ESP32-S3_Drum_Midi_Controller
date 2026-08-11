# ESP32-S3_Drum_Midi_Controller
Convert piezo trigger inputs to Midi percussion control messages over USB<br><br>

## Arduino IDE Configuration<br>
Open the Tools menu in the Arduino IDE and select Board: "ESP32S3 Dev Module"<br>
To use native USB MIDI, a computer must be plugged into the native USB/OTG port on the module.<br>
Most ESP32-S3 DevKit boards feature two USB ports: one labeled UART/COM (connected to an on-board serial chip) and one labeled USB/OTG (connected directly to the ESP32-S3 chip).<br>
Some modules don't label the two USB ports so experimentation is needed until the right port is chosen.<br><br>

Configure these settings in the Arduino IDE Tools menu:<br>


| Menu Option     | Required Setting        | Why it matters                                                                                              |
|-----------------|-------------------------|-------------------------------------------------------------------------------------------------------------|
| USB Mode        | "USB-OTG (TinyUSB)"     | Forces the ESP32-S3 to use the Adafruit TinyUSB stack to act as a class-compliant MIDI device.              |
| USB CDC On Boot | "Enabled"               | Tells the chip to route standard Serial.print() communications over the same USB connection alongside MIDI. |
| Upload Mode     | "USB-OTG CDC (TinyUSB)" | Allows the Arduino IDE to auto-reset the board and flash code directly over the native USB port.            |

<br><br>![PCB](/stuff/Trigger_Flowchart.png)<BR><BR>
