#!/usr/bin/env python3
import time
import board
import adafruit_dht
import digitalio

print("="*60)
print("FINAL DHT COMPREHENSIVE TEST")
print("="*60)
print("Testing ALL combinations on GPIO 17 (Pin 11)\n")

configs = [
    ("DHT22", "use_pulseio=False", lambda: adafruit_dht.DHT22(board.D17, use_pulseio=False)),
    ("DHT22", "use_pulseio=True", lambda: adafruit_dht.DHT22(board.D17, use_pulseio=True)),
    ("DHT11", "use_pulseio=False", lambda: adafruit_dht.DHT11(board.D17, use_pulseio=False)),
    ("DHT11", "use_pulseio=True", lambda: adafruit_dht.DHT11(board.D17, use_pulseio=True)),
]

for sensor_type, config, sensor_func in configs:
    print(f"\n{'='*60}")
    print(f"Testing: {sensor_type} with {config}")
    print(f"{'='*60}")
    
    try:
        sensor = sensor_func()
        print(f"   Sensor object created")
        
        # Try 10 reads with 3 second delays
        for attempt in range(10):
            try:
                temp = sensor.temperature
                hum = sensor.humidity
                
                if temp is not None and hum is not None:
                    print(f"\n{'<‰'*20}")
                    print(f"   SUCCESS! ")
                    print(f"  Sensor Type: {sensor_type}")
                    print(f"  Config: {config}")
                    print(f"  Temperature: {temp}°C")
                    print(f"  Humidity: {hum}%")
                    print(f"{'<‰'*20}\n")
                    
                    # Save the working config to a file
                    with open("working_dht_config.txt", "w") as f:
                        f.write(f"SENSOR_TYPE={sensor_type}\n")
                        f.write(f"USE_PULSEIO={'True' if 'True' in config else 'False'}\n")
                        f.write(f"GPIO=17\n")
                    
                    print("   Configuration saved to 'working_dht_config.txt'")
                    sensor.exit()
                    exit(0)
                else:
                    print(f"  Attempt {attempt+1}/10: Got None values")
                    
            except RuntimeError as e:
                print(f"  Attempt {attempt+1}/10: {e}")
            except Exception as e:
                print(f"  Attempt {attempt+1}/10: Unexpected error: {e}")
            
            time.sleep(3)
        
        sensor.exit()
        print(f"   {sensor_type} with {config} - All attempts failed")
        
    except Exception as e:
        print(f"   Failed to create sensor object: {e}")

print("\n" + "="*60)
print("L ALL CONFIGURATIONS FAILED")
print("="*60)
print("\nPossible issues:")
print("  1. = POWER: Sensor not getting 5V")
print("     ’ Check: VCC connected to Pin 2 or Pin 4 (5V)")
print("     ’ Measure with multimeter if possible")
print("")
print("  2. =' FAULTY SENSOR: The DHT sensor itself is broken")
print("     ’ Try a different DHT sensor")
print("")
print("  3. =Í WRONG PIN: Not actually on GPIO 17")
print("     ’ Verify: DATA wire on Pin 11 (count from top-left)")
print("")
print("  4. = GROUND: Bad ground connection")
print("     ’ Check: GND connected to Pin 6 or Pin 9")
print("")
print("  5. >é INCOMPATIBLE MODULE: Some DHT modules don't work with RPi 5")
print("     ’ The BCM2712 chip timing might be incompatible")
print("="*60)
