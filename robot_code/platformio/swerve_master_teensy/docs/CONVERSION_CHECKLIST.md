# swerve_master.ino Conversion Checklist

## 🎯 **GOAL**: Run `swerve_master.ino` unchanged on both Teensy and Windows simulation

## 📋 **Phase 1: HAL Interface Expansion**

### Core HAL Interfaces
- [x] `ISerial` - Serial communication interface
- [x] `ICANBus` - CAN bus communication interface  
- [x] `IIMU` - IMU sensor interface
- [x] `IRCReceiver` - RC receiver interface
- [ ] `IGPIO` - Digital I/O interface (pinMode, digitalWrite, digitalRead)
- [ ] `IADC` - Analog input interface (analogRead, analogReadResolution)
- [ ] `ITimer` - Timing functions interface (micros, millis, delay, delayMicroseconds)
- [ ] `IWire` - I2C communication interface

### Hardware-Specific HAL Interfaces
- [ ] `IMotorController` - Generic motor control interface
- [ ] `ISteerMotor` - Steering motor interface
- [ ] `IDriveMotor` - Drive motor interface  
- [ ] `IPressurePads` - Weight/pressure pad interface
- [ ] `ILEDStrip` - LED control interface

## 📋 **Phase 2: Arduino Function Replacement**

### Core Arduino Functions
- [ ] Replace `Serial.begin()` → `HAL::serial->begin()`
- [ ] Replace `Serial.print*()` → `HAL::serial->print*()`
- [ ] Replace `pinMode()` → `HAL::gpio->pinMode()`
- [ ] Replace `digitalWrite()` → `HAL::gpio->digitalWrite()`
- [ ] Replace `digitalRead()` → `HAL::gpio->digitalRead()`
- [ ] Replace `analogRead()` → `HAL::adc->analogRead()`
- [ ] Replace `analogReadResolution()` → `HAL::adc->setResolution()`
- [ ] Replace `micros()` → `HAL::timer->micros()`
- [ ] Replace `millis()` → `HAL::timer->millis()`
- [ ] Replace `delay()` → `HAL::timer->delay()`
- [ ] Replace `delayMicroseconds()` → `HAL::timer->delayMicroseconds()`

### Arduino Library Replacements
- [ ] Replace `Wire.h` → `HAL::wire` interface
- [ ] Replace direct CAN calls → `HAL::canBus` interface
- [ ] Replace `Adafruit_BNO055` → `HAL::imu` interface
- [ ] Replace `SbusReceiver` → `HAL::rcReceiver` interface

## 📋 **Phase 3: Hardware Class Conversion**

### Motor Control Classes
- [ ] Convert `Drive` class to use `HAL::motorController`
- [ ] Convert `Steer` class to use `HAL::motorController`
- [ ] Remove direct CAN bus calls from motor classes
- [ ] Update motor initialization to use HAL

### Sensor Classes  
- [ ] Convert `Pads` class to use `HAL::pressurePads`
- [ ] Remove direct ADC calls from sensor classes
- [ ] Update sensor initialization to use HAL

### Communication Classes
- [ ] Convert `SbusReceiver` to use `HAL::rcReceiver`
- [ ] Convert `SwerveTelemetry` to use `HAL::serial`
- [ ] Remove direct UART/Serial calls

### Other Hardware Classes
- [ ] Convert `Lights` class to use `HAL::ledStrip`
- [ ] Remove direct GPIO calls from hardware classes

## 📋 **Phase 4: Build System Updates**

### Include Path Updates
- [ ] Update `swerve_master.ino` to include HAL headers
- [ ] Remove Arduino-specific includes where replaced by HAL
- [ ] Add conditional includes for platform-specific libraries

### Build Configuration
- [ ] Update native build to include `swerve_master.ino`
- [ ] Remove `main_native.cpp` (replaced by actual robot code)
- [ ] Update build filters to include all necessary source files
- [ ] Verify Teensy build still works with HAL changes

## 📋 **Phase 5: Implementation - Real Hardware HAL**

### CAN Bus Implementation
- [ ] Implement `RealCANBus` using FlexCAN_T4
- [ ] Test CAN message sending/receiving
- [ ] Verify motor communication works

### IMU Implementation  
- [ ] Implement `RealIMU` using Adafruit_BNO055
- [ ] Test sensor data reading
- [ ] Verify calibration functions work

### RC Receiver Implementation
- [ ] Implement `RealRCReceiver` using SBUS library
- [ ] Test channel reading
- [ ] Verify failsafe detection

### GPIO/ADC Implementation
- [ ] Implement `RealGPIO` using Arduino digital functions
- [ ] Implement `RealADC` using Arduino analog functions
- [ ] Test pin control and reading

## 📋 **Phase 6: Implementation - Simulation HAL**

### Mock Implementations
- [ ] Implement `SimCANBus` with message logging
- [ ] Implement `SimIMU` with synthetic sensor data
- [ ] Implement `SimRCReceiver` with keyboard/joystick input
- [ ] Implement `SimGPIO` with state tracking
- [ ] Implement `SimADC` with configurable values

### Simulation Features
- [ ] Add WebSocket communication for external control
- [ ] Add configuration file for simulation parameters
- [ ] Add visualization output (optional)

## 📋 **Phase 7: Testing & Validation**

### Teensy Testing
- [ ] Verify `swerve_master.ino` compiles for Teensy
- [ ] Test basic functionality on real hardware
- [ ] Verify all sensors and motors work through HAL
- [ ] Performance testing (ensure no significant overhead)

### Simulation Testing  
- [ ] Verify `swerve_master.ino` compiles for native
- [ ] Test simulation runs without crashes
- [ ] Verify all HAL interfaces provide reasonable mock data
- [ ] Test simulation control and monitoring

### Cross-Platform Validation
- [ ] Verify identical behavior between platforms (where applicable)
- [ ] Test mode switching (teleop, pads, etc.)
- [ ] Validate timing and control loops work correctly
- [ ] Document any platform-specific differences

## 📋 **Phase 8: Documentation & Cleanup**

### Code Documentation
- [ ] Document HAL interface usage in robot code
- [ ] Create examples for each HAL interface
- [ ] Update existing code comments for HAL usage

### User Documentation
- [ ] Update build instructions for converted code
- [ ] Create troubleshooting guide for HAL issues
- [ ] Document simulation setup and usage
- [ ] Create developer guide for adding new HAL interfaces

## 🎯 **Success Criteria**

- [ ] **Same Code**: `swerve_master.ino` runs unchanged on both platforms
- [ ] **Full Functionality**: All robot features work through HAL
- [ ] **Performance**: No significant overhead on real hardware
- [ ] **Maintainability**: Easy to add new platforms or modify implementations
- [ ] **Documentation**: Complete setup and usage guides

## 📊 **Progress Tracking**

- **Phase 1**: ⬜ Not Started (0/8 interfaces)
- **Phase 2**: ⬜ Not Started (0/11 functions)  
- **Phase 3**: ⬜ Not Started (0/8 classes)
- **Phase 4**: ⬜ Not Started (0/4 tasks)
- **Phase 5**: ⬜ Not Started (0/8 implementations)
- **Phase 6**: ⬜ Not Started (0/7 features)
- **Phase 7**: ⬜ Not Started (0/8 tests)
- **Phase 8**: ⬜ Not Started (0/7 docs)

**Overall Progress: 4/71 tasks complete (5.6%)**

---

*This checklist will be updated as tasks are completed. Each checkbox can be marked when the corresponding task is finished and tested.*
