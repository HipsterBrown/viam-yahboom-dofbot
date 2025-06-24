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
	defaultSpeed = 1000
	defaultAccel = 100
	maxSpeed     = 180
	minSpeed     = 3
	maxAccel     = 1145
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
	Speed        float32 `json:"speed_degs_per_sec,omitempty"`
	Acceleration float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`
	DefaultSpeed int     `json:"default_speed,omitempty"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit required (first return) and optional (second return) dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Speed < minSpeed || cfg.Speed > maxSpeed {
		return nil, nil, fmt.Errorf("speed_degs_per_sec must be between %d and %d, you have %f", minSpeed, maxSpeed, cfg.Speed)
	}
	if cfg.Acceleration < 0 || cfg.Acceleration > maxAccel {
		return nil, nil, fmt.Errorf("acceleration_degs_per_sec_per_sec must be between 0 and %d, you have %f", maxAccel, cfg.Acceleration)
	}
	return nil, nil, nil
}

type dofbotArm struct {
	resource.AlwaysRebuild

	name resource.Name

	logger     logging.Logger
	cfg        *Config
	opMgr      *operation.SingleOperationManager
	controller *YahboomServoController

	mu           sync.RWMutex
	speed        float64
	acceleration float64
	defaultSpeed int

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
		name:         rawConf.ResourceName(),
		cfg:          conf,
		opMgr:        operation.NewSingleOperationManager(),
		logger:       logger,
		controller:   controller,
		speed:        float64(conf.Speed),
		acceleration: float64(conf.Acceleration),
		defaultSpeed: conf.DefaultSpeed,
		jointPos:     make([]float64, 5),
		model:        model,
		jointLimits: [][2]float64{
			{0, math.Pi},
			{0, math.Pi},
			{0, math.Pi},
			{0, math.Pi},
			{0, 1.5 * math.Pi},
		},
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}

	if arm.speed == 0 {
		arm.speed = defaultSpeed
	}
	if arm.acceleration == 0 {
		arm.acceleration = defaultAccel
	}
	if arm.defaultSpeed == 0 {
		arm.defaultSpeed = defaultSpeed
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

	clampedPositions := make([]float64, len(values))
	for i, pos := range values {
		if i < len(s.jointLimits) {
			min, max := s.jointLimits[i][0], s.jointLimits[i][1]
			clampedPositions[i] = math.Max(min, math.Min(max, pos))
		} else {
			clampedPositions[i] = pos
		}
	}

	servoAngles := make([]int, 5)
	for i, radians := range clampedPositions {
		degrees := radians * 180.0 / math.Pi
		servoAngles[i] = int(math.Round(degrees))
	}

	gripperPos, err := s.controller.ReadServo(6)
	if err != nil {
		s.logger.Warnf("Failed to read gripper position, using 90: %v", err)
		gripperPos = 90
	}

	maxMovement := 0.0
	for i, target := range servoAngles {
		current := s.jointPos[i] * 180.0 / math.Pi
		movement := math.Abs(float64(target) - current)
		if movement > maxMovement {
			maxMovement = movement
		}
	}

	moveTime := min(max(int(maxMovement*100), 100), 5000)

	allPositions := append(servoAngles, gripperPos)

	if err := s.controller.WriteAllServos(allPositions, moveTime); err != nil {
		return fmt.Errorf("failed to move arm: %w", err)
	}

	s.mu.Lock()
	copy(s.jointPos, clampedPositions)
	s.mu.Unlock()

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
			radians := float64(angle) * math.Pi / 180.0
			positions[i] = referenceframe.Input{Value: radians}
			s.jointPos[i] = radians
		}
		time.Sleep(10 * time.Millisecond)
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
