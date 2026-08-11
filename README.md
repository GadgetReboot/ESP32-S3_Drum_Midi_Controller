# ESP32-S3_Drum_Midi_Controller<br>

Convert piezo trigger inputs to Midi percussion control messages over USB<br><br>

## Hardware Configuration <br>

The ESP32S3 interfaces with 1 or 2 external MCP3008 ADC chips on the SPI bus.  Each chip has 8 analog inputs so a total of up to 16 triggers can be used.<br>
If less triggers are needed, the system can be limited to scanning less channels by changing the CHANNELS_IN_USE = xx; setting in the sketch.<br>
As more channels are scanned, the system has less time to detect trigger peaks on each channel so for optimal performance, set the number of channels according to the number of inputs needed.<br>


### ESP32S3 to MCP3008 ADC Wiring <br>

| ESP32-S3 Pin | Component / Function | Connection on MCP3008 |
|--------------|----------------------|-----------------------|
| GPIO 11      | SPI Clock (SCLK)     | CLK (Both Chips)      |
| GPIO 12      | SPI Data In (MISO)   | DOUT (Both Chips)     |
| GPIO 13      | SPI Data Out (MOSI)  | DIN (Both Chips)      |
| GPIO 10      | Chip Select 1        | /CS on Chip 1         |
| GPIO 9       | Chip Select 2        | /CS on Chip 2         |

## Arduino IDE Configuration <br>

**Notes:** 
- Refer to comments in the top header of the main sketch .INO file for required libraries and the versions of libraries, board files, and Arduino IDE that were tested as working.
- To use native USB MIDI, a computer must be plugged into the native USB/OTG port on the module.<br>
- Most ESP32-S3 DevKit boards feature two USB ports: one labeled UART/COM (connected to an on-board serial chip) and one labeled USB/OTG (connected directly to the ESP32-S3 chip).<br>
- Some modules don't label the two USB ports so experimentation is needed until the right port is chosen.<br><br>

Open the **Tools** menu in the Arduino IDE and select Board: "ESP32S3 Dev Module"<br><br>

Configure these settings in the Arduino IDE Tools menu:<br>


| Menu Option     | Required Setting        | Why it matters                                                                                              |
|-----------------|-------------------------|-------------------------------------------------------------------------------------------------------------|
| USB Mode        | "USB-OTG (TinyUSB)"     | Forces the ESP32-S3 to use the Adafruit TinyUSB stack to act as a class-compliant MIDI device.              |
| USB CDC On Boot | "Enabled"               | Tells the chip to route standard Serial.print() communications over the same USB connection alongside MIDI. |
| Upload Mode     | "USB-OTG CDC (TinyUSB)" | Allows the Arduino IDE to auto-reset the board and flash code directly over the native USB port.            |

## Sketch Flowchart <br>

<br><br>![PCB](/stuff/Trigger_Flowchart.png)<BR><BR>
