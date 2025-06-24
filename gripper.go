package dofbot

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	Gripper = resource.NewModel("hipsterbrown", "dofbot", "gripper")
)

// GripperConfig configuration for the yahboom gripper
type GripperConfig struct {
}

// Validate validates the gripper config
func (cfg *GripperConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

// dofbotGripper represents the yahboom dofbot gripper
type dofbotGripper struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	controller *YahboomServoController
	model      referenceframe.Model

	// State management
	mu       sync.Mutex
	isMoving atomic.Bool
}

func init() {
	resource.RegisterComponent(
		gripper.API,
		Gripper,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newDofbotGripper,
		},
	)
}

func newDofbotGripper(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	controller, err := NewYahboomServoController()

	if err != nil {
		return nil, fmt.Errorf("failed to create servo controller: %w", err)
	}

	g := &dofbotGripper{
		name:       conf.ResourceName(),
		logger:     logger,
		controller: controller,
		model:      referenceframe.NewSimpleModel("dofbot_gripper"),
	}

	return g, nil
}

func (g *dofbotGripper) Name() resource.Name {
	return g.name
}

// Open opens the gripper
func (g *dofbotGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	// Write to servo 6 (gripper servo) - 90 degrees for open
	err := g.controller.WriteServo(6, 90, 1000)
	if err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}

	// Wait for movement to complete
	time.Sleep(1 * time.Second)

	g.logger.Debug("Gripper opened")
	return nil
}

// Grab closes the gripper to grab an object
func (g *dofbotGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	// Write to servo 6 (gripper servo) - 180 degrees for closed
	err := g.controller.WriteServo(6, 180, 1000)
	if err != nil {
		return false, fmt.Errorf("failed to grab with gripper: %w", err)
	}

	// Wait for movement to complete
	time.Sleep(1 * time.Second)

	// Check if something was grabbed by reading the gripper position
	// If the gripper couldn't close fully, it likely grabbed something
	position, err := g.controller.ReadServo(6)
	if err != nil {
		g.logger.Warnf("Failed to read gripper position after grab: %v", err)
		// Assume grab was successful if we can't read position
		return true, nil
	}

	// If the gripper is significantly less than 180 degrees, something is blocking it
	grabbed := position < 170

	if grabbed {
		g.logger.Debug("Gripper successfully grabbed an object")
	} else {
		g.logger.Debug("Gripper closed but may not have grabbed anything")
	}

	return grabbed, nil
}

// Stop stops the gripper movement
func (g *dofbotGripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.isMoving.Store(false)
	// The yahboom controller doesn't have a direct stop command
	// Movement will complete, but we mark as not moving
	g.logger.Debug("Gripper stop requested")
	return nil
}

// IsMoving returns whether the gripper is currently moving
func (g *dofbotGripper) IsMoving(ctx context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

// ModelFrame returns the reference frame model for the gripper
func (g *dofbotGripper) ModelFrame() referenceframe.Model {
	return g.model
}

// Additional helper methods for gripper control

// GetPosition returns the current gripper position (0-180 degrees)
func (g *dofbotGripper) GetPosition(ctx context.Context) (int, error) {
	position, err := g.controller.ReadServo(6)
	if err != nil {
		return 0, fmt.Errorf("failed to read gripper position: %w", err)
	}

	return position, nil
}

// SetPosition sets the gripper to a specific position (0-180 degrees)
func (g *dofbotGripper) SetPosition(ctx context.Context, angle int, moveTime int) error {
	if angle < 0 || angle > 180 {
		return fmt.Errorf("gripper angle must be between 0 and 180 degrees, got %d", angle)
	}

	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	err := g.controller.WriteServo(6, angle, moveTime)
	if err != nil {
		return fmt.Errorf("failed to set gripper position: %w", err)
	}

	// Wait for movement to complete
	time.Sleep(time.Duration(moveTime) * time.Millisecond)

	return nil
}

func (g *dofbotGripper) Close(context.Context) error {
	return nil
}

func (g *dofbotGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *dofbotGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}

func (g *dofbotGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, errors.ErrUnsupported
}

func (g *dofbotGripper) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, errors.ErrUnsupported
}
func (g *dofbotGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, fmt.Errorf("temp hack because of issues")
}
