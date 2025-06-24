package main

import (
	"dofbot"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: dofbot.Arm},
		resource.APIModel{API: gripper.API, Model: dofbot.Gripper},
	)
}
