# ROBOTICS\_DESIGN\_IA1

> Skipy robotic ticketing system — master controller sketch and bitmap header generator.

## Repository Structure

```
ROBOTICS_DESIGN_IA1/
├── BitMapMaker/            # Generate big-digit & logo headers from .bmp files
│   ├── main.ipynb          # Jupyter notebook to convert bitmaps to C headers
│   ├── bitmaps/            # Source .bmp images (digits 0–9)
│   └── digit_headers/      # Output `.h` files for each digit
│
├── Full_Master/            # Arduino master controller sketch
│   ├── Full_Master.ino     # Main Arduino source implementing I²C FSM
│   ├── digit_0.h … digit_9.h
│   ├── BW_Skippy_Logo.h
│   └── Skipy_Star.h
│
├── README.md               # This file
└── .gitignore
```

## Getting Started

### Prerequisites

* **Arduino IDE** v1.8+
* **Python 3** (for BitMapMaker)
* Arduino libraries (install via Library Manager):

  * Wire
  * SoftwareSerial
  * ESC\_POS\_Printer
  * TimeLib (optional, for lunchtime feature)

### Clone the Repository

```bash
git clone https://github.com/<your-org>/ROBOTICS_DESIGN_IA1.git
cd ROBOTICS_DESIGN_IA1
```

### Generate/Update Bitmap Headers

1. Navigate to the BitMapMaker folder:

   ```bash
   cd BitMapMaker
   ```
2. Install Python dependencies:

   ```bash
   pip install notebook numpy pillow
   ```
3. Launch the notebook and run all cells:

   ```bash
   jupyter notebook main.ipynb
   ```
4. Copy the generated header files from `digit_headers/` into `Full_Master/` when complete.

### Build & Upload the Arduino Sketch

1. Open `Full_Master/Full_Master.ino` in the Arduino IDE.
2. Select your board and COM port.
3. Click **Upload**.

## Configuration

* **I²C Addresses** (defined in `Full_Master.ino`):

  ```cpp
  const uint8_t MY_ADDR   = 0b001; // Master
  const uint8_t ACT2_ADDR = 0b010; // Actuation module
  const uint8_t COMM_ADDR = 0b011; // Communication module
  const uint8_t LOC_ADDR  = 0b100; // Localization module
  ```
* **Motor & Sensor Pins**: configured in the motor section of the sketch.
* **Printer Pins**: adjust `PRINTER_RX`/`PRINTER_TX` for your wiring.
* **FSM States**: see the `enum MasterState { ... }` and the `switch(mState)` in `loop()`.

## Lunchtime Trigger (Optional)

You can enable an automatic lunchtime movement by using **TimeLib**. For example, in the `S_IDLE` case:

```cpp
#include <TimeLib.h>

case S_IDLE:
  Serial.println("IDLE");
  // If it is noon, tell localization to start movement
  if (hour() == 12) {
    Serial.println("Lunchtime! Sending START_MOVEMENT_CMD.");
    uint8_t msg = (LOC_ADDR << 5) | START_MOVEMENT_CMD;
    Wire.beginTransmission(LOC_ADDR);
    Wire.write(msg);
    Wire.endTransmission();
  }
  break;
```

## Contributing

1. Fork this repository
2. Create a new branch: `git checkout -b feature/YourFeature`
3. Commit your changes: `git commit -m "Add YourFeature"`
4. Push to your branch: `git push origin feature/YourFeature`
5. Open a Pull Request

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
