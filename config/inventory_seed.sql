-- Project Cloud: Master Inventory Seed Data
-- Database Engine: SQLite3
-- Reference: PROJECT.md Section 7.3 & Appendix B

PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS cabinets (
    cabinet_id INTEGER PRIMARY KEY,
    name VARCHAR(64) NOT NULL,
    zone VARCHAR(32) NOT NULL,
    anchor_tag_id INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS inventory_items (
    item_id INTEGER PRIMARY KEY AUTOINCREMENT,
    sku_code VARCHAR(16) NOT NULL UNIQUE,
    canonical_name VARCHAR(64) NOT NULL UNIQUE,
    aliases_json TEXT NOT NULL,
    stock_qty INTEGER NOT NULL DEFAULT 0,
    unit VARCHAR(16) NOT NULL DEFAULT 'pcs',
    cabinet_id INTEGER NOT NULL REFERENCES cabinets(cabinet_id),
    drawer_code VARCHAR(16) NOT NULL,
    shelf_tier INTEGER NOT NULL,
    anchor_tag_id INTEGER NOT NULL,
    reorder_threshold INTEGER NOT NULL DEFAULT 2,
    custody_bench INTEGER DEFAULT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_inventory_canonical ON inventory_items(canonical_name);
CREATE INDEX IF NOT EXISTS idx_inventory_sku ON inventory_items(sku_code);

-- Seed Cabinets
INSERT OR REPLACE INTO cabinets (cabinet_id, name, zone, anchor_tag_id) VALUES
(1, 'Electronics & Microcontrollers Cabinet', 'Zone A', 1),
(2, 'Sensors, Actuators & Motor Drivers Cabinet', 'Zone A', 2),
(3, 'Power Electronics & Batteries Cabinet', 'Zone A', 3),
(4, 'Precision Hand Tools & Instruments Cabinet', 'Zone B', 4),
(5, 'Rapid 3D Printing & Filament Farm', 'Zone B', 5),
(6, 'Soldering & Fume Extraction Bench', 'Zone C', 6);

-- Seed 50 Inventory Items
INSERT OR REPLACE INTO inventory_items 
(sku_code, canonical_name, aliases_json, stock_qty, unit, cabinet_id, drawer_code, shelf_tier, anchor_tag_id, reorder_threshold)
VALUES
('EL-0001', 'Arduino Uno R3 Development Board', '["arduino", "uno", "atmega328p", "uno r3"]', 12, 'pcs', 1, 'Drawer A', 1, 1, 3),
('EL-0002', 'Arduino Nano V3.0', '["nano", "arduino nano", "atmega328"]', 24, 'pcs', 1, 'Drawer A', 1, 1, 5),
('EL-0003', 'ESP32-WROOM-32D Dual-Core Wi-Fi/BLE', '["esp32", "wroom", "wifi module", "ble module", "nodemcu"]', 19, 'pcs', 1, 'Drawer B', 1, 1, 5),
('EL-0004', 'ESP32-S3-DevKitC-1 with Octal PSRAM', '["esp32-s3", "s3", "octal psram", "ai board"]', 14, 'pcs', 1, 'Drawer B', 1, 1, 4),
('EL-0005', 'Raspberry Pi Pico (RP2040 Dual ARM Cortex)', '["pico", "rp2040", "pi pico"]', 16, 'pcs', 1, 'Drawer C', 2, 1, 4),
('EL-0006', 'Raspberry Pi Pico W (Wireless RP2040)', '["pico w", "wireless pico", "rp2040 wifi"]', 11, 'pcs', 1, 'Drawer C', 2, 1, 3),
('EL-0007', 'STM32F401 Black Pill Development Board', '["black pill", "stm32", "stm32f401", "arm cortex m4"]', 9, 'pcs', 1, 'Drawer D', 2, 1, 2),
('EL-0008', 'Teensy 4.1 High-Performance ARM Cortex-M7', '["teensy", "teensy 4.1", "cortex m7"]', 6, 'pcs', 1, 'Drawer D', 2, 1, 2),
('EL-0009', 'Logic Level Converter 4-Channel Bi-Direct', '["level shifter", "logic converter", "bi directional level converter"]', 28, 'pcs', 1, 'Drawer E', 3, 1, 8),
('EL-0010', 'USB to TTL UART Serial Converter (CP2102)', '["uart converter", "cp2102", "ftdi", "serial adapter", "ttl"]', 15, 'pcs', 1, 'Drawer E', 3, 1, 4),
('PS-0101', 'Resistor Kit 1/4W 1% Metal Film (10Ω-1MΩ)', '["resistors", "resistor pack", "1k resistor", "10k resistor"]', 48, 'pk', 1, 'Drawer F', 3, 1, 10),
('PS-0102', 'Ceramic Capacitor Kit 50V (10pF to 100nF)', '["ceramic capacitors", "capacitors", "100nf", "decoupling cap"]', 32, 'pk', 1, 'Drawer F', 3, 1, 8),
('PS-0103', 'Electrolytic Capacitor Kit (1uF to 1000uF)', '["electrolytic capacitors", "bulk capacitors", "100uf", "1000uf"]', 26, 'pk', 1, 'Drawer G', 4, 1, 6),
('PS-0104', '1N4007 1A Rectifier Diodes (50pk)', '["1n4007", "rectifier diode", "power diode"]', 14, 'pk', 1, 'Drawer G', 4, 1, 4),
('PS-0105', '1N4148 Fast Switching Diodes (50pk)', '["1n4148", "signal diode", "switching diode"]', 12, 'pk', 1, 'Drawer G', 4, 1, 4),
('PS-0106', '2N2222 NPN Bipolar Transistors (TO-92)', '["2n2222", "npn transistor", "bjt", "transistor"]', 65, 'pcs', 1, 'Drawer H', 4, 1, 20),
('PS-0107', 'IRF540N N-Channel Power MOSFET (TO-220)', '["mosfet", "irf540n", "n-channel", "power switch"]', 22, 'pcs', 1, 'Drawer H', 4, 1, 6),
('PS-0108', '5V Active Piezo Buzzer Modules', '["buzzer", "piezo", "beeper", "audio alert"]', 34, 'pcs', 1, 'Drawer H', 4, 1, 10),
('SN-0201', 'BME280 Temperature Humidity Pressure', '["bme280", "weather sensor", "humidity sensor", "temp sensor"]', 8, 'pcs', 2, 'Drawer A', 1, 2, 3),
('SN-0202', 'BMP280 Barometric Pressure Sensor Module', '["bmp280", "altimeter", "barometer", "pressure sensor"]', 14, 'pcs', 2, 'Drawer A', 1, 2, 4),
('SN-0203', 'MPU-6050 6-Axis Accelerometer & Gyro', '["mpu6050", "gyro", "accelerometer", "imu 6 axis"]', 18, 'pcs', 2, 'Drawer B', 1, 2, 5),
('SN-0204', 'BNO055 9-Axis Absolute Orientation IMU', '["bno055", "9-axis imu", "heading sensor", "quaternion sensor"]', 5, 'pcs', 2, 'Drawer B', 1, 2, 2),
('SN-0205', 'VL53L0X Time-of-Flight Laser Distance', '["vl53l0x", "tof sensor", "laser distance", "rangefinder"]', 7, 'pcs', 2, 'Drawer C', 2, 2, 2),
('SN-0206', 'HC-SR04 Ultrasonic Distance Sensor Module', '["hc-sr04", "ultrasonic", "sonar", "distance sensor"]', 21, 'pcs', 2, 'Drawer C', 2, 2, 6),
('SN-0207', 'INA219 High-Side DC Voltage & Current I2C', '["ina219", "current sensor", "power monitor", "ammeter module"]', 11, 'pcs', 2, 'Drawer D', 2, 2, 3),
('SN-0208', 'MQ-2 Combustible Gas & Smoke Sensor', '["mq-2", "gas sensor", "smoke detector", "lpg sensor"]', 8, 'pcs', 2, 'Drawer D', 2, 2, 2),
('AC-0301', 'TowerPro SG90 9g Micro Servo Motor', '["sg90", "micro servo", "servo 9g", "mini servo"]', 26, 'pcs', 2, 'Drawer E', 3, 2, 8),
('AC-0302', 'MG996R High-Torque Metal Gear Servo Motor', '["mg996r", "metal gear servo", "high torque servo"]', 12, 'pcs', 2, 'Drawer E', 3, 2, 4),
('AC-0303', 'NEMA 17 Stepper Motor 42mm (1.5A 40 N·cm)', '["nema 17", "stepper motor", "3d printer motor"]', 9, 'pcs', 2, 'Drawer F', 3, 2, 3),
('AC-0304', 'A4988 Stepper Motor Driver Carrier Module', '["a4988", "stepper driver", "pololu driver"]', 24, 'pcs', 2, 'Drawer F', 3, 2, 6),
('AC-0305', 'TMC2209 SilentStepStick Motor Driver Board', '["tmc2209", "silent stepper", "trinamic driver"]', 16, 'pcs', 2, 'Drawer F', 3, 2, 4),
('AC-0306', 'L298N Dual H-Bridge Motor Driver Module', '["l298n", "motor driver", "dual h-bridge", "dc motor driver"]', 14, 'pcs', 2, 'Drawer G', 4, 2, 4),
('AC-0307', '12V High-Torque Planetary Micro DC Motor', '["dc motor", "gear motor", "planetary motor"]', 8, 'pcs', 2, 'Drawer G', 4, 2, 2),
('PW-0401', 'LM2596 Adjustable DC-DC Step-Down Buck', '["lm2596", "buck converter", "step down regulator"]', 18, 'pcs', 3, 'Drawer A', 1, 3, 6),
('PW-0402', 'XL6009 High-Efficiency Step-Up Boost Conv', '["xl6009", "boost converter", "step up regulator"]', 14, 'pcs', 3, 'Drawer A', 1, 3, 4),
('PW-0403', 'TP4056 1A Li-Ion Battery Charger Board', '["tp4056", "lipo charger", "18650 charger board"]', 32, 'pcs', 3, 'Drawer B', 1, 3, 10),
('PW-0404', '18650 Protected Li-Ion Cell 3000mAh 3.7V', '["18650", "li-ion battery", "3.7v battery", "lithium cell"]', 22, 'pcs', 3, 'Drawer B', 1, 3, 6),
('PW-0405', 'MeanWell 12V 10A Enclosed Power Supply', '["meanwell 12v", "12v power supply", "switching power supply"]', 4, 'pcs', 3, 'Drawer C', 2, 3, 1),
('PW-0406', '24V 5A Industrial Benchtop Power Brick', '["24v supply", "bench power supply", "power brick 24v"]', 3, 'pcs', 3, 'Drawer C', 2, 3, 1),
('TL-0501', '150mm Stainless-Steel Digital Calipers', '["calipers", "digital vernier", "measuring calipers"]', 4, 'pcs', 4, 'Drawer A', 1, 4, 1),
('TL-0502', 'Automatic Wire Stripper & Multi-Crimper', '["wire stripper", "crimper", "cable stripper"]', 6, 'pcs', 4, 'Drawer A', 1, 4, 2),
('TL-0503', 'Pinecil Smart Portable Soldering Iron', '["pinecil", "portable iron", "ts100", "usb-c soldering iron"]', 3, 'pcs', 4, 'Drawer B', 2, 4, 1),
('TL-0504', 'Hakko FX-888D Digital Soldering Station', '["soldering station", "hakko", "fx-888d", "iron station"]', 2, 'pcs', 4, 'Drawer B', 2, 4, 1),
('TL-0505', 'Knipex Precision Flush Cutters (125mm)', '["flush cutters", "diagonal cutters", "knipex", "wire snips"]', 5, 'pcs', 4, 'Drawer C', 3, 4, 2),
('TL-0506', 'Wiha Precision Screwdriver Set (24-Piece)', '["screwdrivers", "wiha set", "precision bits", "torx drivers"]', 3, 'pcs', 4, 'Drawer C', 3, 4, 1),
('FL-0601', 'PolyLite PLA+ 1.75mm Filament (Black, 1kg)', '["pla", "pla filament", "black filament", "3d printing filament"]', 8, 'sp', 5, 'Rack Tier 1', 1, 5, 2),
('FL-0602', 'PolyMax PETG-CF Carbon Composite (1kg)', '["petg-cf", "carbon fiber filament", "petg", "strong filament"]', 4, 'sp', 5, 'Rack Tier 1', 1, 5, 1),
('FL-0603', 'Polymaker PolyFlex TPU95A Flexible (1kg)', '["tpu", "flexible filament", "rubber filament", "tpu95a"]', 3, 'sp', 5, 'Rack Tier 2', 2, 5, 1),
('FL-0604', 'Hardened Steel 0.4mm 3D Printer Nozzles', '["nozzles", "0.4mm nozzle", "hardened nozzle", "e3d nozzle"]', 16, 'pcs', 5, 'Bin Tier 3', 3, 5, 4),
('FL-0605', 'Textured PEI Spring-Steel Magnetic Sheets', '["pei sheet", "build plate", "spring steel bed", "print bed"]', 5, 'pcs', 5, 'Bin Tier 3', 3, 5, 2);
