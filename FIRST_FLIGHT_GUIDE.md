# First Flight Guide: F450 + Arduino (YMFC-AL based)

Follow these steps **in order**. Don't skip a check: each one catches a mistake that would otherwise flip or crash the drone on take-off. Every step says what you should see and what to do if you don't.

> **Props stay OFF until step 9.** Hold the frame or strap it down whenever the motors can spin.

---

## Step 0: Parts check

| Part | Requirement |
|---|---|
| Board | Arduino **Uno or Nano (ATmega328P, 16 MHz)**. A Mega, ESP32 or Nano Every will NOT work. |
| Gyro | **MPU-6050** (GY-521 board). Other gyros aren't supported. |
| ESCs | 4× standard PWM ESCs (SimonK/BLHeli are fine). They must accept 250 Hz. |
| Motors / props | F450-class: 2212 ~920 KV with 10×4.5 props, 2 CW + 2 CCW |
| Battery | 3S LiPo (11.1 V nominal, 12.6 V full) |
| Radio | Transmitter + receiver with **PWM outputs**, at least 4 channels |
| Extras | 5 V supply for the Arduino (an ESC's BEC or a UBEC), LED + 330 Ω resistor, battery divider (see below) |

**Software:** Arduino IDE with **Arduino AVR Boards 1.8.3 or newer** (Tools → Board → Boards Manager). With older versions the sketches won't compile; that's on purpose, so the I2C safety timeout can't be silently left out.

---

## Step 1: Wiring

| Arduino pin | Connect to |
|---|---|
| D8, D9, D10, D11 | Receiver channel signal wires (any order: setup learns it) |
| D4 | ESC 1, **front-right**, motor spins **CCW** |
| D5 | ESC 2, **rear-right**, motor spins **CW** |
| D6 | ESC 3, **rear-left**, motor spins **CCW** |
| D7 | ESC 4, **front-left**, motor spins **CW** |
| A4 / A5 | MPU-6050 SDA / SCL (VCC → 5 V, GND → GND) |
| D12 | LED (+), through a 330 Ω resistor to GND |
| A0 | Battery divider: battery + → 1N4001 diode → **1.5 kΩ** → A0 → **1 kΩ** → GND (12.6 V reads about 4.7 V, which is what the code expects) |
| GND | Common ground for ESCs, receiver, MPU-6050 and battery divider |

- Mount the MPU-6050 **flat, near the centre of the frame, on foam or rubber**. Hard-mounting it is the most common cause of a wobbly quad.
- Keep the I2C wires (A4/A5) short and away from the ESC power wires.
- "Front" is whichever way you decide. Mark it on the frame now.

**Check:** power the Arduino from USB only (no battery). The board's LED should light, and nothing should get hot.

---

## Step 2: Setup program (`ymfc_setup`)

1. Turn the transmitter on, and make sure the receiver is bound and powered.
2. Upload `ymfc_setup/ymfc_setup.ino`. Open the Serial Monitor at **57600 baud**.
3. Follow the prompts:

| It asks | You do |
|---|---|
| "Place all sticks and subtrims in the center position" | All sticks centred, throttle centred too (just for this step), all trims at 0 |
| "Move the throttle stick to full throttle and back to center" | Throttle full up, then back to centre |
| "Move the roll stick to simulate left wing up..." | Roll stick to the **right**, then back |
| "Move the pitch stick to simulate nose up..." | Pitch stick **pulled back** (towards you), then back |
| "Move the yaw stick to simulate nose right..." | Yaw stick to the **right**, then back |
| "Gently move all the sticks simultaneously to their extends" | Stir all sticks to every corner, then centre them |
| "Lift the left side of the quadcopter to a 45 degree angle" | Tilt the frame so the **left** side goes up |
| "Lift the nose of the quadcopter to a 45 degree angle" | Tilt the **front** up |
| "Rotate the nose of the quadcopter 45 degree to the right" | Turn it flat, nose to the **right** |
| "LED test" | The LED on D12 should light up |

**Expected end:** `Setup is finished.`

| Error | Meaning / fix |
|---|---|
| ERROR 1 | One or more receiver channels missing. It tells you which input (8-11). Check the wire and the binding. |
| ERROR 2 | No stick movement seen. Move the stick fully, within 30 s. |
| ERROR 3 / 10 | No MPU-6050 found / wrong gyro type. Check the 5 V, GND, SDA→A4 and SCL→A5 wiring. |
| ERROR 4 / 7 | Tilt not detected. Tilt further (45°) and faster, within 10 s. |
| ERROR 6 | Not all four sticks were assigned. Redo it, moving each stick separately. |
| ERROR 9 | The MPU-6050 stopped answering. Loose wire or bad board. |

---

## Step 3: ESC calibration (`esc_calibration`), PROPS OFF

1. Upload `esc_calibration/esc_calibration.ino`. **Unplug USB and the battery.**
2. Transmitter on, **throttle at FULL**.
3. Plug in the battery. The ESCs beep (the max-throttle tone).
4. Move the throttle to **minimum**. The ESCs beep again (the min tone and an "armed" melody).
5. ESC calibration is done. Leave the battery plugged in and connect USB for the next steps.

**Check:** send nothing yet and slowly raise the throttle: all 4 motors should start together, smoothly, at the same point. Then throttle back to minimum.

---

## Step 4: Check the sticks (`r`)

Type `r` in the Serial Monitor and press Enter.

| Move | You must see |
|---|---|
| Roll stick right | `Roll:>>>`, value goes **up** to about 2000 |
| Pitch stick forward (push) | `Pitch:^^^`, value goes **down** to about 1000 |
| Throttle up | `Throttle:^^^`, value goes **up** to about 2000 |
| Yaw stick right | `Yaw:>>>`, value goes **up** to about 2000 |
| Sticks centred | All around 1500 (1480-1520), throttle low about 1000 |

If any one is reversed or swapped, **re-run step 2**. Don't use the transmitter's reverse switch after setup.

---

## Step 5: Check the angles (`a`)

Type `a`. Keep the quad still for the 2-second gyro calibration.

| Tilt | You must see |
|---|---|
| Nose **up** | `Pitch` **positive** |
| Right side **down** | `Roll` **positive** |
| Rotate nose right | `Yaw` positive while it's turning |
| Flat on the table | Pitch and Roll near 0 |

**Accelerometer level offsets:** put the quad on a *really* level surface (check with a spirit level) and note the Pitch and Roll readings. Write them into `flight_controller.ino`:

```cpp
float acc_pitch_offset = 0.0;   // put your "Pitch" reading here
float acc_roll_offset = 0.0;    // put your "Roll" reading here
```

---

## Step 6: Motor direction and order (`1`-`4`), PROPS OFF

Type `1`, set the throttle to minimum, then raise it a little. **Only the named motor may spin:**

| Key | Motor | Position | Must spin |
|---|---|---|---|
| `1` | ESC 1 | front-right | **counter-clockwise** (seen from above) |
| `2` | ESC 2 | rear-right | **clockwise** |
| `3` | ESC 3 | rear-left | **counter-clockwise** |
| `4` | ESC 4 | front-left | **clockwise** |

- Wrong motor spins → swap the signal wires on D4-D7.
- Wrong direction → swap any **two** of the three wires between that motor and its ESC.

---

## Step 7: Props on, balance them (`1`-`5`)

Fit the props: CCW props on motors 1 and 3, CW props on motors 2 and 4. The **thick leading edge** of each blade must point in the direction of rotation. Tighten the prop nuts.

**Strap the quad down** (or hold it very firmly), then type `1`. Raise the throttle to about hover (half stick). The Serial Monitor prints a vibration number. Balance the prop (tape on the lighter blade, or sand the heavier one) until the number is as low as you can get it. Repeat for `2`, `3` and `4`, then `5` for all motors together.

> Why this matters: in the simulator, the code tolerated about 8× the vibration of balanced props before flight got noticeably worse, but unbalanced props are still the #1 reason a home-built quad wobbles.

---

## Step 8: Failsafe setup

1. **Receiver failsafe:** in your transmitter/receiver settings, set failsafe to **"no pulses"** (also called "no signal" or "hold off"). If the receiver keeps sending the last stick positions after losing the transmitter, the Arduino can't detect the loss.
2. In `flight_controller.ino`, choose what happens when the signal is lost:
   - `failsafe_throttle = 1000;` (default): motors stop immediately. Safest for the first flights, close to the ground.
   - `failsafe_throttle = 1300;` (a bit **below** your hover throttle): slow, level descent for `failsafe_descent_ms` (5 s), then the motors stop. Set this once you know your hover throttle (step 10).

---

## Step 9: Upload the flight controller and bench test, PROPS OFF

1. Remove the props again. Upload `flight_controller/flight_controller.ino`.
2. Plug in the battery with the transmitter on and the **throttle low**. The LED lights, blinks fast during gyro calibration (keep the quad still for about 8 s), then goes **off**.
3. **Arm:** throttle down + yaw left, then yaw back to centre. All 4 motors should idle slowly.
4. **Tilt test (most important):** with a little throttle, tilt the quad by hand:
   - Right side down → the **right** motors must speed up.
   - Nose down → the **front** motors must speed up.

   If the **wrong** side speeds up, **do not fly**: re-run step 2 (the gyro axes are wrong).
5. **Failsafe test:** armed, small throttle, switch the **transmitter off**. All motors must stop within about 0.1 s.
6. **Disarm:** throttle down + yaw right. Motors stop.

| LED | Meaning |
|---|---|
| Blinking fast at start-up | Calibrating (normal) |
| Off | Ready |
| Slow blink (2 Hz) | Battery low: land / charge |
| Solid on in flight | Loop too slow or **gyro failure**. Motors are stopped; power-cycle and check the MPU-6050 wiring. |
| Solid on, never goes off at start-up | Setup not done, or the MPU-6050 isn't answering |
| Very fast blink forever | Wrong gyro type stored. Re-run setup. |

---

## Step 10: First flight

- Open space, no people, no wind. Props on, tightened.
- Arm, then raise the throttle **slowly** until it just lifts. Hop to about 0.5 m and set it down. Repeat a few times.
- Note the throttle stick position where it hovers. That's your **hover throttle**. You can now set `failsafe_throttle` about 50 below it.
- If it **drifts slowly** in one direction: re-check the accelerometer offsets (step 5) and the centre trims.
- If it **flips instantly** on take-off: STOP. Motor order, direction or gyro axes are wrong. Redo steps 4-6 and the tilt test in step 9.

---

## Step 11: Tuning (only if needed)

The default PID gains were tested in the simulator on 23 different F450 builds (0.9-1.9 kg, weak to strong motors, slow ESCs, strong wind, unbalanced props) and stayed stable on all of them. Most F450s fly fine on the defaults. If yours doesn't:

| Symptom | Change (in `flight_controller.ino`) |
|---|---|
| Fast shaking / buzzing | Lower `pid_d_gain_roll` (for example 18 → 14). Check prop balance first. |
| Slow wobbling, overshoots | Lower `pid_p_gain_roll` (1.3 → 1.0) |
| Feels sloppy, drifts back slowly after gusts | Raise `pid_i_gain_roll` slightly (0.04 → 0.05) |
| Yaw drifts / slow to stop | Raise `pid_p_gain_yaw` (4.0 → 5.0) |

Change one value at a time, by about 20%, and test with short hops. Pitch uses the roll values automatically.

> **Not for small frames:** the simulator showed these gains oscillate on a quad with about a third of an F450's inertia (a 250-size racer). This code and its gains are for F450-sized frames.
