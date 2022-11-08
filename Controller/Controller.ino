// #include <ArduPID.h>

const byte PWM_PIN = 3;
const byte DIR_PIN = 5;
const byte FORWARD = 1;
const byte REVERSE = abs(FORWARD - 1);
byte reverseDelay = 10;
byte val = 0;
byte dir = FORWARD;

// ArduPID pid;

void setup()
{
    Serial.begin(115200);

    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    analogWrite(PWM_PIN, 0);
    digitalWrite(DIR_PIN, dir);

    Serial.println("Init complete");
}

void loop()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readString();
        input.trim();
        Serial.println(input);
        if (input.substring(0, 1) == "!")
        {
            reverseDelay = input.substring(1).toInt();
            Serial.println("~Set reverseDelay to " + (String)reverseDelay + "ms");
        }
        else if (input.substring(0, 1) == "%")
        {
            float incoming = parseCommand(input.substring(1).toFloat());
            byte newDir = (incoming >= 0) ? FORWARD : REVERSE;
            byte newVal = (byte)abs((255 * incoming));
            Serial.print(">Set duty cycle to " + (String)(incoming * 100.0) + "% or ");
            Serial.println((newDir == FORWARD ? "" : "-") + (String)newVal + "/255");
            updateEngineSpeed(newVal, newDir);
        }
        else
        {
            int incoming = input.toInt();
            byte newDir = (incoming >= 0) ? FORWARD : REVERSE;
            byte newVal = (byte)abs(incoming);
            Serial.println(">Set duty cycle to " + (String)(newDir == FORWARD ? "" : "-") + (String)(newVal) + "/255");
            updateEngineSpeed(newVal, newDir);
        }
    }
}

void updateEngineSpeed(byte newVal, byte newDir)
{
    if (newVal == 0 || newDir == dir)
    {
        val = newVal;
        analogWrite(PWM_PIN, newVal);
    }
    else
    {
        analogWrite(PWM_PIN, 0);
        delay(reverseDelay);
        dir = newDir;
        digitalWrite(DIR_PIN, newDir);
        updateEngineSpeed(newVal, newDir);
    }
}

float parseCommand(float input)
{
    if (abs(input) > 100)
    {
        while (abs(input) > 1)
        {
            input = input / 10;
        }
    }
    else
    {
        input = input / 100;
    }
    return input;
}