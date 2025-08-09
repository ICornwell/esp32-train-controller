# IR Sensor Wiring Guide

## Simple IR Break-beam Sensors (Photointerrupters)

### Recommended Components:
- **GP1A57HRJ00F** - Sharp IR photointerrupter (3.3V compatible)
- **LTH-301-32** - Lite-On slotted photointerrupter
- **MOC7811** - Fairchild slotted optical switch

### Basic Wiring:
```
IR LED Side:           ESP32:
+3.3V ---|>|--- GPIO (optional, always-on)
         LED
         Resistor (220Ω)
         
Photodetector Side:    ESP32:
Collector --------> GPIO32/33/34/35 (input pins)
Emitter ---------> GND
VCC ------------> 3.3V
```

### Sensor Placement Strategy:

1. **Platform Stop (GPIO32)** - End of platform
   - Triggers automatic stop for passenger service
   - Perfect for timed station stops

2. **Siding Entry (GPIO33)** - Before siding turnout
   - Reduces speed for safe siding entry
   - Could trigger point motor for automatic routing

3. **Junction Detection (GPIO34)** - At track junctions
   - Log train movements for signaling
   - Could control signal lights (red/green LEDs)

4. **End Bumper (GPIO35)** - End of track
   - Emergency stop to prevent derailment
   - Safety backup for runaway trains

### Cost Comparison:
- **IR Photointerrupter**: £1-3 each
- **PN532 RFID Reader**: £8-15 each
- **Total for 4 IR sensors**: £4-12 vs £32-60 for 4 RFID readers!

### Advanced Features:
- **Speed Detection**: Two sensors close together can measure train speed
- **Direction Detection**: Sequence of sensor triggers shows direction
- **Occupancy Detection**: Know which track sections have trains
- **Automatic Signaling**: Red/green signals based on track occupancy

### Integration Benefits:
1. Use **one RFID reader** at the control point to assign train ID
2. Use **multiple IR sensors** for automatic track control
3. Cost-effective expansion across entire layout
4. Simple wiring (just 3 wires per sensor vs complex I2C/UART)
5. Fast response time (immediate vs 100ms+ for RFID polling)

### Future Expansion:
```
Control Panel:     Track Sensors:     Actions:
RFID Reader  ----> IR Platform   ----> Auto Stop
   |          \--> IR Siding     ----> Speed Limit
   |           \-> IR Junction   ----> Signal Control
   |            \> IR Bumper     ----> Emergency Stop
   |
   v
Train Database:
- Train ID
- Speed Limits  
- Sound Files
- Lighting Control
```

This hybrid approach gives you the best of both worlds:
- **Intelligence** from RFID (what train, what sounds, what capabilities)  
- **Automation** from IR sensors (where is it, what should happen)
- **Economy** by minimizing expensive RFID readers
