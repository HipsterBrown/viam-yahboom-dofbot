package dofbot

import (
	"context"
	_ "embed"
	"encoding/json"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

var (
	Arm              = resource.NewModel("hipsterbrown", "dofbot", "arm")
	errUnimplemented = errors.New("unimplemented")
)

const (
	minLimit = -math.Pi / 2
	maxLimit = math.Pi / 2
)

//go:embed dofbot.json
var modelJson []byte

func init() {
	resource.RegisterComponent(arm.API, Arm,
		resource.Registration[arm.Arm, *Config]{
			Constructor: newDofbotArm,
		},
	)
}

type Config struct {
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit required (first return) and optional (second return) dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

type dofbotArm struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	cfg        *Config
	opMgr      *operation.SingleOperationManager
	controller *YahboomServoController

	mu          sync.RWMutex
	moveLock    sync.Mutex
	isMoving    atomic.Bool
	jointPos    []float64
	model       referenceframe.Model
	jointLimits [][2]float64

	cancelCtx  context.Context
	cancelFunc func()
}

func makeModelFrame() (referenceframe.Model, error) {
	m := &referenceframe.ModelConfigJSON{OriginalFile: &referenceframe.ModelFile{Bytes: modelJson, Extension: "json"}}
	err := json.Unmarshal(modelJson, m)
	if err != nil {
		return nil, errors.Wrap(err, "failed to unmarshal json file")
	}

	return m.ParseConfig("dofbot")
}

func newDofbotArm(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	controller, err := NewYahboomServoController()
	if err != nil {
		return nil, fmt.Errorf("failed to create servo controller: %w", err)
	}

	model, err := makeModelFrame()
	if err != nil {
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	arm := &dofbotArm{
		name:       rawConf.ResourceName(),
		cfg:        conf,
		opMgr:      operation.NewSingleOperationManager(),
		logger:     logger,
		controller: controller,
		jointPos:   make([]float64, 5),
		model:      model,
		jointLimits: [][2]float64{
			{minLimit, maxLimit},
			{minLimit, maxLimit},
			{minLimit, maxLimit},
			{minLimit, maxLimit},
			{minLimit, math.Pi},
		},
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}

	return arm, nil

}

func NewArm(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (arm.Arm, error) {

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &dofbotArm{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	return s, nil
}

func (s *dofbotArm) Name() resource.Name {
	return s.name
}

func (s *dofbotArm) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (arm.Arm, error) {
	panic("not implemented")
}

func (s *dofbotArm) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}

	pose, err := referenceframe.ComputeOOBPosition(s.model, inputs)
	if err != nil {
		return nil, fmt.Errorf("failed to compute end position: %w", err)
	}

	return pose, nil
}

func (s *dofbotArm) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if err := motion.MoveArm(ctx, s.logger, s, pose); err != nil {
		return err
	}
	return nil
}

func (s *dofbotArm) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	s.moveLock.Lock()
	defer s.moveLock.Unlock()

	s.isMoving.Store(true)
	defer s.isMoving.Store(false)

	values := make([]float64, len(positions))
	for i, input := range positions {
		values[i] = input.Value
	}

	// Validate input ranges and clamp positions
	clampedPositions := make([]float64, len(values))
	for i, pos := range values {
		min, max := s.jointLimits[i][0], s.jointLimits[i][1]

		// Validate and clamp the position
		if pos < min || pos > max {
			s.logger.Warnf("Joint %d position %.3f rad (%.1f°) out of range [%.3f, %.3f] rad ([%.1f°, %.1f°]), clamping",
				i+1, pos, pos*180/math.Pi, min, max, min*180/math.Pi, max*180/math.Pi)
		}
		clampedPositions[i] = math.Max(min, math.Min(max, pos))
	}

	// Convert radians to servo angles with translation
	servoAngles := make([]int, 5)
	for i, radians := range clampedPositions {
		if i >= 5 {
			break // Only process first 5 joints
		}

		// Convert radians to degrees
		degrees := radians * 180.0 / math.Pi
		if i != 0 {
			degrees = -degrees
		}
		servoAngle := degrees + 90

		servoAngles[i] = int(math.Round(servoAngle))

		// Additional validation to ensure servo angles are in valid ranges
		if i == 4 && (servoAngles[i] < 0 || servoAngles[i] > 270) {
			s.logger.Errorf("Joint 5 servo angle %d out of range [0, 270]", servoAngles[i])
			servoAngles[i] = int(math.Max(0, math.Min(270, float64(servoAngles[i]))))
		} else if i < 4 && (servoAngles[i] < 0 || servoAngles[i] > 180) {
			s.logger.Errorf("Joint %d servo angle %d out of range [0, 180]", i+1, servoAngles[i])
			servoAngles[i] = int(math.Max(0, math.Min(180, float64(servoAngles[i]))))
		}
	}

	// Read current gripper position
	gripperPos, err := s.controller.ReadServo(6)
	if err != nil {
		s.logger.Warnf("Failed to read gripper position, using 90: %v", err)
		gripperPos = 90
	}

	// Calculate movement time based on maximum joint movement
	maxMovement := 0.0
	for i, target := range servoAngles {
		if i < len(s.jointPos) {
			// Convert current position from radians to servo angle for comparison
			currentRadians := s.jointPos[i]
			currentDegrees := currentRadians * 180.0 / math.Pi
			if i != 0 {
				currentDegrees = -currentDegrees
			}
			servoAngle := currentDegrees + 90

			movement := math.Abs(float64(target) - servoAngle)
			if movement > maxMovement {
				maxMovement = movement
			}
		}
	}

	// Calculate move time based on degrees per second requirements
	// Min speed: 10 degrees per second (slower) = more time
	// Max speed: 30 degrees per second (faster) = less time
	maxTimeMs := int(maxMovement * 1000 / 10) // Min speed: 10 deg/sec
	minTimeMs := int(maxMovement * 1000 / 30) // Max speed: 30 deg/sec

	// Ensure moveTime is within bounds
	moveTime := max(minTimeMs, min(maxTimeMs, 5000))

	// Combine servo angles with gripper position
	allPositions := append(servoAngles, gripperPos)

	// Send command to controller
	if err := s.controller.WriteAllServos(allPositions, moveTime); err != nil {
		return fmt.Errorf("failed to move arm: %w", err)
	}

	// Update cached joint positions
	s.mu.Lock()
	copy(s.jointPos, clampedPositions)
	s.mu.Unlock()

	// Wait for movement to complete
	time.Sleep(time.Duration(moveTime) * time.Millisecond)

	return nil
}

func (s *dofbotArm) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	for _, jointPositions := range positions {
		if err := s.MoveToJointPositions(ctx, jointPositions, extra); err != nil {
			return err
		}

		if ctx.Err() != nil {
			return ctx.Err()
		}
	}
	return nil
}

func (s *dofbotArm) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	positions := make([]referenceframe.Input, 5)

	for i := range 5 {
		angle, err := s.controller.ReadServo(i + 1)
		if err != nil {
			s.logger.Warnf("failed to read servo %d, using cached value: %v", i+1, err)
			positions[i] = referenceframe.Input{Value: s.jointPos[i]}
		} else {
			// Apply reverse translation from servo angles to joint angles
			jointDegrees := float64(angle) - 90.0 // Maps 0° to -90°, 270° to 180°
			if i != 0 {
				jointDegrees = -jointDegrees
			}

			// Convert degrees to radians
			radians := jointDegrees * math.Pi / 180.0

			positions[i] = referenceframe.Input{Value: radians}
			s.jointPos[i] = radians
		}
		time.Sleep(100 * time.Millisecond)
	}
	return positions, nil
}

func (s *dofbotArm) Stop(ctx context.Context, extra map[string]interface{}) error {
	s.isMoving.Store(false)
	return nil
}

func (s *dofbotArm) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return s.model, nil
}

func (s *dofbotArm) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

func (s *dofbotArm) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *dofbotArm) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	panic("not implemented")
}

func (s *dofbotArm) IsMoving(ctx context.Context) (bool, error) {
	return s.isMoving.Load(), nil
}

func (s *dofbotArm) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := s.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (s *dofbotArm) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
