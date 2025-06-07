# ROBOTICS\_DESIGN\_IA1

> Skipy robotic ticketing system — master controller sketch and bitmap header generator.

## Repository Structure

```
ROBOTICS_DESIGN_IA1/
├── Discover/            # Legacy tests and experimental code
│   └── (old sketches, prototypes, data)
├── Develop/             # Intermediate iterations and proofs of concept
│   └── (versioned code drafts, notes)
├── Deliver/             # Final deliverables
│   ├── BitMapMaker/     # Generate big-digit & logo headers from .bmp files
│   │   ├── main.ipynb   # Jupyter notebook to convert bitmaps to C headers
│   │   ├── bitmaps/     # Source .bmp images (digits 0–9)
│   │   └── digit_headers/ # Output `.h` files for each digit
│   └── Full_Master/     # Arduino master controller sketch
│       ├── Full_Master.ino # Main Arduino source implementing I²C FSM
│       ├── digit_0.h ... digit_9.h
│       ├── BW_Skippy_Logo.h
│       └── Skipy_Star.h
├── LICENSE              # MIT License
├── README.md            # This file
└── .gitignore
```

ROBOTICS\_DESIGN\_IA1/
├── BitMapMaker/            # Generate big-digit & logo headers from .bmp files
│   ├── main.ipynb          # Jupyter notebook to convert bitmaps to C headers
│   ├── bitmaps/            # Source .bmp images (digits 0–9)
│   └── digit\_headers/      # Output `.h` files for each digit
│
├── Full\_Master/            # Arduino master controller sketch
│   ├── Full\_Master.ino     # Main Arduino source implementing I²C FSM
│   ├── digit\_0.h ... digit\_9.h
│   ├── BW\_Skippy\_Logo.h
│   └── Skipy\_Star.h
│
├── README.md               # This file
└── .gitignore

````

---

## Getting Started

### Prerequisites

- **Arduino IDE** v1.8+
- **Python 3** (for BitMapMaker)
- Arduino libraries (install via Library Manager):
  - Wire
  - SoftwareSerial
  - ESC\_POS\_Printer
  - TimeLib (optional, for lunchtime feature)

### Clone the Repository

```bash
git clone https://github.com/<your-org>/ROBOTICS_DESIGN_IA1.git
cd ROBOTICS_DESIGN_IA1
````

### Generate/Update Bitmap Headers

1. Navigate to the `BitMapMaker` folder:

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
4. Copy the generated header files from `digit_headers/` into `Full_Master/`.

### Build & Upload the Arduino Sketch

1. Open `Full_Master/Full_Master.ino` in the Arduino IDE.
2. Select your board and COM port.
3. Click **Upload**.

---

## Configuration

* **I²C Addresses** (in `Full_Master.ino`):

  ```cpp
  const uint8_t MY_ADDR   = 0b001; // Master
  const uint8_t ACT2_ADDR = 0b010; // Actuation module
  const uint8_t COMM_ADDR = 0b011; // Communication module
  const uint8_t LOC_ADDR  = 0b100; // Localization module
  ```
* **Motor & Sensor Pins**: configured in the motor section of the sketch.
* **Printer Pins**: adjust `PRINTER_RX`/`PRINTER_TX` as needed.
* **FSM States**: see the `enum MasterState { ... }` and the `switch (mState)` block.

---

## Lunchtime Trigger (Optional)

To automatically send `START_MOVEMENT_CMD` at noon, add this to the `S_IDLE` case (requires TimeLib):

```cpp
#include <TimeLib.h>

case S_IDLE:
  Serial.println("IDLE");
  if (hour() == 12) {
    Serial.println("🕛 Lunchtime! Sending START_MOVEMENT_CMD.");
    uint8_t msg = (LOC_ADDR << 5) | START_MOVEMENT_CMD;
    Wire.beginTransmission(LOC_ADDR);
    Wire.write(msg);
    Wire.endTransmission();
  }
  break;
```

---

## Contributing

1. Fork this repository.
2. Create a new branch:

   ```bash
   ```

git checkout -b feature/YourFeature

````
3. Commit your changes:
```bash
git commit -m "Add YourFeature"
````

4. Push to your branch:

   ```bash
   ```

git push origin feature/YourFeature

```
5. Open a Pull Request.

---

---

## Message Table

Below is the table of I²C message exchanges (format `<SOURCE:TARGET:CMD:VALUE>`):

### System Boot

| Source | Target | Command         | Value |
| ------ | ------ | --------------- | ----- |
| MASTER | ALL    | ASK_READY_CMD   | 00011 |
| MODULE | MASTER | TELL_READY_CMD  | 11100 |

### Idle / Active Mode

| Source | Target | Command       | Value |
| ------ | ------ | ------------- | ----- |
| MASTER | ALL    | GO_IDLE_CMD   | 10111 |
| MASTER | ALL    | GO_ACTIVE_CMD | 11000 |

### Master ↔ Localization

| Source       | Target | Command                | Value |
| ------------ | ------ | ---------------------- | ----- |
| MASTER       | LOC    | START_MOVEMENT_CMD     | 00010 |
| LOCALIZATION | MASTER | STARTED_MOVEMENT_CMD   | 00100 |
| LOCALIZATION | MASTER | ENDED_MOVEMENT_CMD     | 00111 |

### Master ↔ Communication

| Source        | Target | Command                   | Value |
| ------------- | ------ | ------------------------- | ----- |
| COMMUNICATION | MASTER | STARTED_INTERACTION_CMD   | 01000 |
| COMMUNICATION | MASTER | ENDED_INTERACTION_CMD     | 01111 |
| COMMUNICATION | MASTER | NOTIFY_HEADTOUCH_CMD      | 01001 |
| MASTER        | COMM   | NOTIFY_LUCKYBALL_CMD      | 01010 |
| MASTER        | COMM   | NOTIFY_NORMALBALL_CMD     | 01011 |

### Master ↔ Actuator 2

| Source    | Target | Command                  | Value |
| --------- | ------ | ------------------------ | ----- |
| MASTER    | ACT2   | START_FLOW_CMD           | 10001 |
| ACTUATOR2 | MASTER | STARTED_FLOW_CMD         | 10010 |
| ACTUATOR2 | MASTER | ENDED_FLOW_CMD           | 10011 |
| ACTUATOR2 | MASTER | DETECTED_LUCKYBALL_CMD   | 11010 |
| ACTUATOR2 | MASTER | DETECTED_NORMALBALL_CMD  | 11011 | 

## License

This project is licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

```
