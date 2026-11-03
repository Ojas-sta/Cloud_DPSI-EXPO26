# LEDCommand.msg: Addressable NeoPixel halo lighting commands
# Reference: PROJECT.md Section 6.2 & Section 7.8

uint8 MODE_OFF = 0
uint8 MODE_SOLID = 1
uint8 MODE_BREATHE = 2
uint8 MODE_CHASE = 3
uint8 MODE_STROBE = 4
uint8 MODE_CASCADE = 5

uint8 mode
uint8 red                       # 0-255
uint8 green                     # 0-255
uint8 blue                      # 0-255
float32 frequency_hz            # Cycle rate (e.g., 0.5 Hz for breathing, 4 Hz for alert strobe)
float32 duration_sec            # 0.0 for infinite until next command
