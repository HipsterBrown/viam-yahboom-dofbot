package dofbot

import (
	"fmt"
	"time"

	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/host/v3"
)

const (
	// I2C address of the arm controller
	controllerAddr = 0x15
)

// YahboomServoController handles I2C communication with the yahboom servo controller
type YahboomServoController struct {
	dev i2c.Dev
}

// NewYahboomServoController creates a new servo controller instance
func NewYahboomServoController() (*YahboomServoController, error) {
	// Initialize periph.io
	if _, err := host.Init(); err != nil {
		return nil, fmt.Errorf("failed to initialize periph.io: %w", err)
	}

	// Open the I2C bus
	bus, err := i2creg.Open("")
	if err != nil {
		return nil, fmt.Errorf("failed to open I2C bus: %w", err)
	}

	// Create device handle
	dev := &i2c.Dev{Addr: controllerAddr, Bus: bus}

	return &YahboomServoController{dev: *dev}, nil
}

// ReadServo reads the current position of a servo (1-6)
func (c *YahboomServoController) ReadServo(id int) (int, error) {
	if id < 1 || id > 6 {
		return 0, fmt.Errorf("servo id must be 1-6, got %d", id)
	}

	// First write 0x0 to register (id + 0x30) to request position
	register := byte(id + 0x30)
	if err := c.dev.Tx([]byte{register, 0x0}, nil); err != nil {
		return 0, fmt.Errorf("I2C write error for servo %d: %w", id, err)
	}

	time.Sleep(3 * time.Millisecond)

	// Now read 2 bytes from the same register
	read := make([]byte, 2)
	if err := c.dev.Tx([]byte{register}, read); err != nil {
		return 0, fmt.Errorf("I2C read error for servo %d: %w", id, err)
	}

	// Convert bytes to position (little endian from Python: pos >> 8 & 0xff) | (pos << 8 & 0xff00)
	// This swaps the bytes, so we need to handle the byte order correctly
	pos := (int(read[1]) << 8) | int(read[0])

	// Apply the same byte swap as in Python
	pos = ((pos >> 8) & 0xff) | ((pos << 8) & 0xff00)

	if pos == 0 {
		return 0, fmt.Errorf("servo %d returned invalid position", id)
	}

	// Convert raw position to angle based on servo type
	var angle int
	if id == 5 {
		// Servo 5 has different range (0-270°)
		angle = (270-0)*(pos-380)/(3700-380) + 0
		if angle > 270 || angle < 0 {
			return 0, fmt.Errorf("servo %d angle out of range: %d", id, angle)
		}
	} else {
		// Standard servos (0-180°)
		angle = (180-0)*(pos-900)/(3100-900) + 0
		if angle > 180 || angle < 0 {
			return 0, fmt.Errorf("servo %d angle out of range: %d", id, angle)
		}
	}

	// Invert angle for servos 2, 3, 4
	if id == 2 || id == 3 || id == 4 {
		angle = 180 - angle
	}

	return angle, nil
}

// WriteServo writes a position to a specific servo
func (c *YahboomServoController) WriteServo(id, angle, moveTime int) error {
	if id < 1 || id > 6 {
		return fmt.Errorf("servo id must be 1-6, got %d", id)
	}

	// Validate angle limits
	if id == 5 && (angle > 270 || angle < 0) {
		return fmt.Errorf("servo %d angle out of range: %d (0-270)", id, angle)
	} else if id != 5 && (angle > 180 || angle < 0) {
		return fmt.Errorf("servo %d angle out of range: %d (0-180)", id, angle)
	}

	var pos int

	// Handle servo 5 (different range)
	if id == 5 {
		pos = (3700-380)*(angle-0)/(270-0) + 380
	} else {
		// Invert angle for servos 2, 3, 4
		adjustedAngle := angle
		if id == 2 || id == 3 || id == 4 {
			adjustedAngle = 180 - angle
		}
		pos = (3100-900)*(adjustedAngle-0)/(180-0) + 900
	}

	// Split position and time into high/low bytes
	valueH := byte((pos >> 8) & 0xFF)
	valueL := byte(pos & 0xFF)
	timeH := byte((moveTime >> 8) & 0xFF)
	timeL := byte(moveTime & 0xFF)

	// Write to register (0x10 + id) with position and time data
	register := byte(0x10 + id)
	data := []byte{valueH, valueL, timeH, timeL}
	write := append([]byte{register}, data...)

	if err := c.dev.Tx(write, nil); err != nil {
		return fmt.Errorf("failed to write servo %d: %w", id, err)
	}

	return nil
}

// WriteAllServos writes positions to all 6 servos simultaneously
func (c *YahboomServoController) WriteAllServos(positions []int, moveTime int) error {
	if len(positions) != 6 {
		return fmt.Errorf("expected 6 positions, got %d", len(positions))
	}

	// Validate all positions
	for i, pos := range positions {
		if i == 4 && (pos > 270 || pos < 0) { // servo 5 (index 4)
			return fmt.Errorf("servo %d position out of range: %d (0-270)", i+1, pos)
		} else if i != 4 && (pos > 180 || pos < 0) {
			return fmt.Errorf("servo %d position out of range: %d (0-180)", i+1, pos)
		}
	}

	// Convert positions to raw values
	data := make([]byte, 12) // 6 servos * 2 bytes each

	for i, angle := range positions {
		var pos int
		servoID := i + 1

		if servoID == 5 {
			pos = (3700-380)*(angle-0)/(270-0) + 380
		} else {
			adjustedAngle := angle
			if servoID == 2 || servoID == 3 || servoID == 4 {
				adjustedAngle = 180 - angle
			}
			pos = (3100-900)*(adjustedAngle-0)/(180-0) + 900
		}

		data[i*2] = byte((pos >> 8) & 0xFF) // high byte
		data[i*2+1] = byte(pos & 0xFF)      // low byte
	}

	// Write time data to register 0x1e
	timeData := []byte{byte((moveTime >> 8) & 0xFF), byte(moveTime & 0xFF)}
	timeWrite := append([]byte{0x1e}, timeData...)
	if err := c.dev.Tx(timeWrite, nil); err != nil {
		return fmt.Errorf("failed to write time data: %w", err)
	}

	// Write position data to register 0x1d
	posWrite := append([]byte{0x1d}, data...)
	if err := c.dev.Tx(posWrite, nil); err != nil {
		return fmt.Errorf("failed to write position data: %w", err)
	}

	return nil
}
