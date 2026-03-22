# Files

include\SwerveTelemetry.h
src\SwerveTelemetry.cpp

# Updates

- I don't want this to be specific to swerve data. I want a general telemetry system class/lib thats able to send the following types of data:
    - int
    - float
    - double
    - bool
    - string
    - array of any of the above

- Each type of data should have a method for sending it. For example:
    - sendInt(String name, int value)
    - sendFloat(String name, float value)
    - sendDouble(String name, double value)
    - sendBool(String name, bool value)
    - sendString(String name, String value)
    - sendIntArray(String name, int[] value)
    - sendFloatArray(String name, float[] value)
    - sendDoubleArray(String name, double[] value)
    - sendBoolArray(String name, bool[] value)

- These methods just add to the UDP packet, but the packet is not sent until send() is called. This way we can send multiple types of data in one packet.

- The data should be sent in a JSON format. For example if .sendInt("a", 1) and .sendFloat("b", 2.0) and .sendString("c", "hello") were called, the JSON object would look like:

```json
{
    "a": 1,
    "b": 2.0,
    "c": "hello"
}
```

- https://github.com/bblanchon/ArduinoJson should be used as the json library