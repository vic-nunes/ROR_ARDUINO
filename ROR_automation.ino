#include "Arduino.h"
/**
 * ROR Automation - Arduino Roof Control System
 * 
 * LICENSE:
 *    MIT License
 *
 *   Copyright (c) 2025 Vitor Nunes
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 * 
 * DISCLAIMER:
 * This software is provided "as is" without any warranty of any kind, express or implied.
 * Users are solely responsible for:
 * - Testing this code in their specific environment before deployment
 * - Understanding the hardware requirements and safety implications
 * - Ensuring proper safety mechanisms are in place (especially regarding roof movement)
 * - Any damage, injury, or loss resulting from the use or misuse of this software
 * 
 * The author/developer assumes no liability for:
 * - Incorrect implementation or configuration
 * - Hardware failures or malfunctions
 * - Safety hazards related to automated roof control
 * - Data loss or system failures
 * 
 * By using this code, you acknowledge that you understand these risks and accept
 * full responsibility for its use in your application.
 *
 *
 *  To be used ONLY in conjunction with VINU ROR ASCOM driver available in: https://github.com/vic-nunes/ROR_ASCOM
 */

//
// Hardware: Arduino Uno/CLONE
//

String serialin; // Incoming serial data
String str; // String to store return state (for the 'get#' command)

// --- CONTROL PINS ---
#define STOP_PIN 4       // Stop signal output pin
#define MOTOR_PIN 5      // Motor relay control pin (pulse)
#define MOTOR_PIN_OPEN 6      // Motor relay control pin (pulse)
#define MOTOR_PIN_CLOSE 7      // Motor relay control pin (pulse)
#define STATUS_LED_PIN 8 // Status LED (ON = intermediate / moving)
#define BUTTON_PIN 9     // Manual control button input pin

// Input pins (NC sensors using INPUT_PULLUP: LOW = activated/limit)
// 'opened' and 'closed' are end-stop sensors.
// 'safe' is a normally-open safety sensor on pin 13. A change (closed -> triggered)
// should stop the roof if it's moving.
#define SENSOR_OPENED 11        // Roof opened sensor
#define SENSOR_CLOSED 12        // Roof closed sensor
#define SAFETY_SENSOR 13          // Safety sensor (NO) on pin 13

// --- STATE & LOCK VARIABLES ---
// TRUE if the last pulse started movement and it hasn't been stopped yet.
bool isMoving = false;
// Button state variable for edge detection
int lastButtonState = HIGH;

// State constants
const int MOVING = 1;
const int STOPPED = 0;
int laststatus = 99;

enum Position
{
    POS_UNKNOWN, POS_OPENED, POS_CLOSED
};
Position currentPosition = POS_UNKNOWN;
Position lastPosition = POS_UNKNOWN;

// --- Direction Tracking ---
enum Direction
{
    DIR_NONE, DIR_OPENING, DIR_CLOSING
};
Direction currentDirection = DIR_NONE;
Direction lastDirection = DIR_NONE;

// Stores the direction of movement when an unsafe event occurs
Direction directionBeforeUnsafe = DIR_NONE;

enum Safety
{
    SAF_SAFE, SAF_UNSAFE
};

Safety currentSafety = SAF_UNSAFE; // Flag to indicate we are in an unsafe state waiting to become SAFETY_SENSOR
Safety previousSafety = SAF_UNSAFE;

// To prevent double pulses from the manual button (1 seconds lockout)
#define BUTTON_LOCKOUT_MS 1000UL
unsigned long lastPulseTime = 0;

String command_buffer = "";

// Sensor states tracking
int lastSafeState = HIGH;    // Safety sensor previous state
int lastOpenState = HIGH;    // Open sensor previous state
int lastCloseState = HIGH;   // Close sensor previous state

// --- MOTOR PULSE FUNCTION ---
// Sends a pulse to the relay to START/STOP/REVERSE and updates movement state.
// The function now receives the intended Direction so the sketch can track
// whether we are opening or closing when the pulse is sent.
void
pulseMotor (int dir)
{
    // Low-level relay pulse
    digitalWrite (MOTOR_PIN, HIGH);
    // 100ms pulse to ensure motor controller registers the action.
    delay (100);
    digitalWrite (MOTOR_PIN, LOW);

    // Update timing and movement state
    lastPulseTime = millis ();

    isMoving = !isMoving;            // Movement reversed because of this pulse

    if (isMoving)
    {
        laststatus = MOVING;
    }
    else
    {
        laststatus = STOPPED;
    }

    currentDirection = (Direction) dir; // Remember direction at the moment of pulse
    previousSafety = SAF_SAFE;
    currentSafety = SAF_SAFE;
}

// Sends a pulse to open the roof (pin 6)
void
pulseMotorOpen ()
{
    // Low-level relay pulse for open
    digitalWrite (MOTOR_PIN_OPEN, HIGH);
    // 100ms pulse to ensure motor controller registers the action.
    delay (300);
    digitalWrite (MOTOR_PIN_OPEN, LOW);

    // Update timing and movement state
    lastPulseTime = millis ();

    isMoving = true;
    laststatus = MOVING;

    currentDirection = DIR_OPENING;  // Set direction to opening
    currentPosition = POS_UNKNOWN;
    lastPosition = POS_CLOSED;
    previousSafety = SAF_SAFE;
    currentSafety = SAF_SAFE;

    // 1 sec to allow to leave the sensors
    delay (700);
}

// Sends a pulse to close the roof (pin 7)
void
pulseMotorClose ()
{
    // Low-level relay pulse for close
    digitalWrite (MOTOR_PIN_CLOSE, HIGH);
    // 100ms pulse to ensure motor controller registers the action.
    delay (300);
    digitalWrite (MOTOR_PIN_CLOSE, LOW);

    // Update timing and movement state
    lastPulseTime = millis ();

    isMoving = true;
    laststatus = MOVING;

    currentDirection = DIR_CLOSING;  // Set direction to closing
    currentPosition = POS_UNKNOWN;
    lastPosition = POS_OPENED;
    previousSafety = SAF_SAFE;
    currentSafety = SAF_SAFE;

    // 1 sec to allow to leave the sensors
    delay (700);
}

// Sends a pulse on the stop pin when sensors change state
void
updateStopSignal ()
{
    digitalWrite (STOP_PIN, HIGH);
    delay (300);  // 100ms pulse
    digitalWrite (STOP_PIN, LOW);

    isMoving = false;
    laststatus = STOPPED;
}

// Function to split a string by a delimiter
void
splitString (String input, String delimiter, String* outputArray, int arraySize)
{
    int startIndex = 0;
    int delimiterIndex = 0;
    int arrayIndex = 0;

    while (delimiterIndex != -1 && arrayIndex < arraySize)
    {
        delimiterIndex = input.indexOf (delimiter, startIndex);

        if (delimiterIndex == -1)
        {
            // Last element
            outputArray[arrayIndex] = input.substring (startIndex);
        }
        else
        {
            // Extract substring between delimiters
            outputArray[arrayIndex] = input.substring (startIndex,
                                                       delimiterIndex);
            startIndex = delimiterIndex + delimiter.length ();
        }

        arrayIndex++;
    }
}

// --- FUNCTION TO PROCESS SERIAL COMMANDS ---
void
processSerialCommand (String cmd)
{
    if (cmd == "")
    {
        return;
    }

    bool openedState = (digitalRead (SENSOR_OPENED) == LOW);
    bool closedState = (digitalRead (SENSOR_CLOSED) == LOW);

    bool safeState = digitalRead (SAFETY_SENSOR) == HIGH;

    if (cmd == "stop")
    {
        updateStopSignal ();
    }
    else if (cmd == "open")
    {
        if (!openedState && safeState)
        { // Only open if not already at the SENSOR_OPENED limit
            pulseMotorOpen ();
        }
    }
    else if (cmd == "close")
    {
        if (!closedState && safeState)
        { // Only close if not already at the closed limit
            pulseMotorClose ();
        }
    }
    else if (cmd == "get")
    {
        str = "entered get";

        if (currentSafety == SAF_UNSAFE)
        {
            // If the roof was opening, report it as closed to force a close command.
            if (directionBeforeUnsafe == DIR_OPENING)
            {
                str = "closed,unsafe,not_moving_c#";
            }
            // If the roof was closing, report it as opened to force an open command.
            else if (directionBeforeUnsafe == DIR_CLOSING)
            {
                str = "opened,unsafe,not_moving_o#";
            }
            else
            {
                // by default if we have unknow position, we state that it's closed
                str = "closed,unsafe,not_moving_c#";
            }
            // Send the manipulated state and reset the flag.
            Serial.println (str.c_str ());

            return; // Exit to prevent normal 'get' processing for this cycle.
        }
        str = "passed unsafe";

        // --- Logic for state recovery after an unsafe event ---
        if (currentSafety == SAF_SAFE && previousSafety == SAF_UNSAFE)
        {
            // If the roof was opening, report it as closed to force a close command.
            if (directionBeforeUnsafe == DIR_OPENING)
            {
                str = "closed,safe,not_moving_c#";
                // Send the manipulated state and reset the flag.
                Serial.println (str.c_str ());
                //   directionBeforeUnsafe = DIR_NONE; // Reset after sending the state once.
                return;// Exit to prevent normal 'get' processing for this cycle.
            }
            // If the roof was closing, report it as opened to force an open command.
            else if (directionBeforeUnsafe == DIR_CLOSING)
            {
                str = "opened,safe,not_moving_o#";
                // Send the manipulated state and reset the flag.
                Serial.println (str.c_str ());
                //   directionBeforeUnsafe = DIR_NONE; // Reset after sending the state once.
                return;// Exit to prevent normal 'get' processing for this cycle.
            }

        }

        // --- Normal 'get' Processing ---
        str = "";
        // PART 1: Position state (opened, closed, opening, closing, unknown)
        if (openedState)
        {
            str += "opened,";
        }
        else if (closedState)
        {
            str += "closed,";
        }
        else
        {
            // If not at a limit, position is unknown
            // Check movement direction to report opening/closing or unknown
            if (isMoving)
            {
                if (currentDirection == DIR_OPENING)
                {
                    str += "opening,";
                }
                else if (currentDirection == DIR_CLOSING)
                {
                    str += "closing,";
                }
            }
            else
            {
                if (lastPosition == POS_OPENED)
                {
                    str += "opened,";
                }
                else if (lastPosition == POS_CLOSED)
                {
                    str += "closed,";
                }
            }
        }

        // PART 2: Safety state
        if (safeState)
        {
            str += "safe,";
        }
        else
        {
            str += "unsafe,";
        }

        // PART 3: Movement state (moving, not_moving_o, not_moving_c, not_moving )
        if (openedState)
        {
            str += "not_moving_o#";
        }
        else if (closedState)
        {
            str += "not_moving_c#";
        }
        else
        {
            // Position is unknown (neither sensor active)
            if (isMoving)
            {
                str += "moving#";
            }
            else
            {
                if (lastPosition == POS_OPENED)
                {
                    str += "not_moving_o#";
                }
                else if (lastPosition == POS_CLOSED)
                {
                    str += "not_moving_c#";
                }
            }
        }

        // Initial pulls
        String parts[3];  // Array to store (position, safety, movement)
        splitString (str, ",", parts, 3);

        if (parts[0].length () == 0 || parts[2].length () == 0)
        {
            if (safeState)
            {
                str = "closed,safe,not_moving_c#";
            }
            else
            {
                str = "closed,unsafe,not_moving_c#";
            }
        }
        Serial.println (str.c_str ());
    }
}

void
setup ()
{
    Serial.begin (9600);
    while (Serial.available ())
    {
        Serial.read ();
    }

    pinMode (MOTOR_PIN, OUTPUT);
    pinMode (STOP_PIN, OUTPUT);
    pinMode (MOTOR_PIN_CLOSE, OUTPUT);
    pinMode (MOTOR_PIN_OPEN, OUTPUT);

    pinMode (STATUS_LED_PIN, OUTPUT);

    pinMode (SENSOR_OPENED, INPUT_PULLUP);
    pinMode (SENSOR_CLOSED, INPUT_PULLUP);
    pinMode (SAFETY_SENSOR, INPUT_PULLUP);
    pinMode (BUTTON_PIN, INPUT_PULLUP);

    digitalWrite (STOP_PIN, LOW);  // Initialize stop pin as LOW

    digitalWrite (MOTOR_PIN, LOW);

    // Protocol: send the identification string VINU#
    Serial.flush ();
    delay (500);
    Serial.println ("VINU#");
}

void
handleSerial ()
{
    // 3. Serial command processing
    while (Serial.available () > 0)
    {
        char incomingByte = Serial.read ();
        command_buffer += incomingByte;
        // Process the buffer when the delimiter '#' is found
        if (incomingByte == '#')
        {
            String clean_command = command_buffer;
            clean_command.trim ();
            clean_command.replace ("#", ""); // Remove the delimiter

            // Init command, reset arduino.
            if (clean_command.equalsIgnoreCase ("Init"))
            {
                ((void
                (*) ()) 0) (); // Chama o endereço 0 para forçar o reset
            }

            processSerialCommand (clean_command);

            command_buffer = ""; // Clear buffer for the next command
        }
    }
}

void
inferPosition (int currentCloseRead, int currentOpenRead, int currentSafeState)
{
    // --- Infer direction from sensor state changes ---
    if (isMoving)
    {
        // If we just left the 'closed' sensor, we must be opening.
        if (lastCloseState == LOW && currentCloseRead == HIGH)
        {
            currentDirection = DIR_OPENING;
            currentPosition = POS_UNKNOWN;
            lastPosition = POS_CLOSED;
        }
        else
        // If we just left the 'opened' sensor, we must be closing.
        if (lastOpenState == LOW && currentOpenRead == HIGH)
        {
            currentDirection = DIR_CLOSING;
            currentPosition = POS_UNKNOWN;
            lastPosition = POS_OPENED;
        }
    }

    // Detect transition to UNSAFE (HIGH -> LOW)
    if (lastSafeState == HIGH && currentSafeState == LOW)
    {
        // An unsafe event happened while moving. Store the direction.
        directionBeforeUnsafe = currentDirection;
        currentSafety = SAF_UNSAFE; // We are now waiting for it to become safe again
        previousSafety = SAF_SAFE;
    }
    else
    // Detect transition back to SAFE (LOW -> HIGH)
    if (lastSafeState == LOW && currentSafeState == HIGH)
    {
        // If we were waiting for this, the special state will be sent on the next 'get'.
        // If not, directionBeforeUnsafe is DIR_NONE and nothing special happens.
        currentSafety = SAF_SAFE; // We are now waiting for it to become safe again
        previousSafety = SAF_UNSAFE;
    }

    // Update last known sensor states for the next loop iteration
    lastSafeState = currentSafeState;
    lastOpenState = currentOpenRead;
    lastCloseState = currentCloseRead;
}

void
loop ()
{
    //   delay (700);
    int currentOpenRead = digitalRead (SENSOR_OPENED);
    int currentCloseRead = digitalRead (SENSOR_CLOSED);
    int currentSafeRead = digitalRead (SAFETY_SENSOR);

    // 1. Check all sensors
    int currentSafeState = digitalRead (SAFETY_SENSOR);

    // Check if any sensor is active
    bool anySensorActive = (digitalRead (SENSOR_OPENED) == LOW
            || digitalRead (SENSOR_CLOSED) == LOW);

    // Update movement state and LED based on all sensors
    if (digitalRead (SAFETY_SENSOR) == LOW)
    {
        updateStopSignal ();
        if (isMoving)
        {
            digitalWrite (STATUS_LED_PIN, LOW);
        }

        // --- Infer direction from sensor state changes ---
        inferPosition (currentCloseRead, currentOpenRead, currentSafeState);

        currentSafety = SAF_UNSAFE;
        handleSerial ();
        return;
    }

    currentSafety = SAF_SAFE;

    if (anySensorActive)
    {
        if (currentOpenRead == LOW)
        {
            currentPosition = POS_OPENED;
            lastPosition = POS_UNKNOWN;
        }
        else if (currentCloseRead == LOW)
        {
            currentPosition = POS_CLOSED;
            lastPosition = POS_UNKNOWN;
        }

        if (isMoving)
        {
            digitalWrite (STATUS_LED_PIN, LOW);

            updateStopSignal ();
            previousSafety = SAF_SAFE;

            handleSerial ();
            return;
        }
    }
    else
    {
        // Do not assume movement just because no end-stop is active.
        // Movement state should be updated only on pulses (start) or stops.
        // Reflect current moving state on the status LED.
        digitalWrite (STATUS_LED_PIN, isMoving ? HIGH : LOW);
    }

    // 2. Manual button control (pin 9)
    int currentButtonState = digitalRead (BUTTON_PIN);

    if (lastButtonState == HIGH && currentButtonState == LOW
            && millis () - lastPulseTime > BUTTON_LOCKOUT_MS)
    {
        if (currentSafeState == HIGH)
        {
            previousSafety = SAF_SAFE;

            // Decide desired direction for the manual button press.
            Direction desiredDir = DIR_NONE;

            // If roof is at closed limit, a press should open it.
            if (digitalRead (SENSOR_CLOSED) == LOW)
            {
                desiredDir = DIR_OPENING;
            }
            // If roof is at opened limit, a press should close it.
            else if (digitalRead (SENSOR_OPENED) == LOW)
            {
                desiredDir = DIR_CLOSING;
            }
            else
            {
                // If neither limit is active, choose the opposite of current
                // direction if known, otherwise default to opening.
                if (currentDirection == DIR_CLOSING)
                    desiredDir = DIR_OPENING;
                else
                    desiredDir = DIR_CLOSING;
            }

            // Send a pulse with the chosen direction.
            if (desiredDir == DIR_OPENING)
            {
                pulseMotorOpen ();
            }
            else if (desiredDir == DIR_CLOSING)
            {
                pulseMotorClose ();
            }
        }
    }
    lastButtonState = currentButtonState;

    // --- Infer direction from sensor state changes ---
    //   inferPosition (currentCloseRead, currentOpenRead, currentSafeState);

    // 3. Serial command processing
    handleSerial ();

    // Ensure the buffer is cleared if it grows too large
    if (command_buffer.length () > 50)
    {
        command_buffer = "";
    }
}

