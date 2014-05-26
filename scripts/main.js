window.addEventListener("load", function() {
	// Set up world: aversaries and agent
	var WIDTH = 10;
	var HEIGHT = 10;
	worldPositions = []

	var Location = function (x, y) {
		if (x && y) {
			return {
				x: x,
				y: y
			};
		} else {
			return worldPositions[Math.floor(Math.random() * worldPositions.length)]
		}
	}

	for (var i = 0; i < WIDTH; i++) {
		for (var j = 0; j < HEIGHT; j++) {
			worldPositions[worldPositions.length] = new Location(i, j);
		}
	}

	var manhattanDistance = function (pos1, pos2) {
		return Math.abs(pos1.x - pos2.x) + Math.abs(pos1.y - pos2.y)
	}

	/****** Motion Models *****/

	var randomMotionModel = function (adversaries, robot, location) {
		var model = [];
		for (var i = 0; i < worldPositions.length; i++) {
			var position = worldPositions[i];
			var dx = Math.abs(position.x - location.x);
			var dy = Math.abs(position.y - location.y);
			if (manhattanDistance(position, location) == 1) {
				model[model.length] = position;
			}
		}
		for (var i = 0; i < model.length; i++) {
			model[i] = {
				position: model[i],
				probability: 1 / model.length
			}
		}
		return model;
	}

	/****** Observation Models *****/

	var uniformDistribution = function () {
		var numPositions = 0;
		var distribution = [];
		for (var i = 0; i < worldPositions.length; i++) {
			position = worldPositions[i];
			distribution[distribution.length] = {
				position: position,
				probability: 1
			};
			numPositions++;
		}
		for (i in distribution) {
			distribution[i].probability = 1/numPositions;
		}
		return distribution;
	}

	var singleAgentDistanceDistribution = function (location, distance) {
		var distribution = [];
		var total = 0;
		for (var i = 0; i < worldPositions.length; i++) {
			var position = worldPositions[i];
			var distance = manhattanDistance(position, location);
			if (Math.abs(location - distance) == 0) {
				distribution[distribution.length] = {
					position: position,
					probability: 0.4
				}
				total += 0.4;
			}
			if (Math.abs(location - distance) == 1) {
				distribution[distribution.length] = {
					position: position,
					probability: 0.2
				}
				total += 0.2;
			}
			if (Math.abs(location - distance) == 2) {
				distribution[distribution.length] = {
					position: position,
					probability: 0.1
				}
				total += 0.1;
			}
		}
		for (var i = 0; i < distribution.length; i++) {
			distribution[i].probability /= total;
		}
		return distribution;
	}

	var singleClosestAdversaryObservation = function (adversaries, robot) {
		var closestAdversaryIndex = 0;
		var distance = manhattanDistance(robot.location, adversaries[0].location);
		for (var i = 0; i < adversaries.length; i++) {
			var newDistance = manhattanDistance(robot.location, adversaries[i].location);
			if (newDistance < distance) {
				closestAdversaryIndex = i;
				distance = newDistance;
			}
		}
		var observations = [];
		for (var i = 0; i < adversaries.length; i++) {
			if (i == closestAgentIndex) {
				observations[i] = singleAgentDistanceDistribution(robot.location, distance);
			} else {
				observations[i] = uniformDistribution();
			}
		}
		return observations;
	}

	var singleFarthestAdversaryObservation = function (adversaries, robot) {
		var farthestAdversaryIndex = 0;
		var distance = manhattanDistance(robot.location, adversaries[0].location);
		for (var i = 0; i < adversaries.length; i++) {
			var newDistance = manhattanDistance(robot.location, adversaries[i].location);
			if (newDistance > distance) {
				farthestAdversaryIndex = i;
				distance = newDistance;
			}
		}
		var observations = [];
		for (var i = 0; i < adversaries.length; i++) {
			if (i == farthestAgentIndex) {
				observations[i] = singleAgentDistanceDistribution(robot.location, distance);
			} else {
				observations[i] = uniformDistribution();
			}
		}
		return observations;
	}

	var singleRandomAdversaryObservation = function (adversaries, robot) {
		var randAdversaryIndex = Math.floor(Math.random() * adversaries.length);
		var distance = manhattanDistance(robot.location, adversaries[randAdversaryIndex].location);
		var observations = [];
		for (var i = 0; i < adversaries.length; i++) {
			if (i == randAdversaryIndex) {
				observations[i] = singleAgentDistanceDistribution(robot.location, distance);
			} else {
				observations[i] = uniformDistribution();
			}
		}
		return observations;
	}

	var multipleAgentObservation = function (adversaries, robot) {
		var observations = [];
		for (var i = 0; i < adversaries.length; i++) {
			var distance = manhattanDistance(robot.location, adversaries[i].location)
			observations[i] = singleAgentDistanceDistribution(robot.location, distance);
		}
		return observations;
	}

	/****** Tracking Methods *****/
	// Distributions are arrays with each element having position and probability already normalized
	// Positions can be repeated in distributions
	var sampleFromDistribution = function (distribution) {
		var rand = Math.random();
		var i = -1;
		while (rand >= 0) {
			i++;
			rand -= distribution[i].probability;
		}
		return distribution[i].position;
	}

	var getLocationProbability = function (location, distribution) {
		var probability = 0;
		for (var i = 0; i < distribution.length; i++) {
			var position = distribution[i].position;
			if (position.x == location.x && position.y == location.y) {
				probability += distribution[i].probability;
			}
		}
		return probability;
	}

	var ExactInference = function (distributions, observationModel) {
		// Private variables
		var beliefs = [];
		var observationModelFunction = observationModel;

		// Create beliefs if given prior distributions
		if (distributions) {
			for (var i = 0; i < distributions.length; i++) {
				beliefs[i] = distributions[i].slice();
			}
		}

		var setObservationModel = function (modelFunction) {
			observationModelFunction = modelFunction;
		}

		// TODO Exact Inference (observe/elapseTime)
		var observe = function (adversaries, robot) {
			var observations = observationModelFunction(adversaries, robot);
			for (var i = 0; i < adversaries.length; i++) {
				var observation = observations[i];
				var belief = beliefs[i];
				var totalWeight = 0;
				for (var p = 0; p < belief.length; p++) {
					var particle = belief[p];
					belief[p] = {
						location: particle,
						probability: getLocationProbability(particle, observation)
					};
					totalWeight += belief[p].probability;
				}
				// Normalize
				for (var p = 0; p < belief.length; p++) {
					belief[p].probability /= totalWeight;
				}

				// Sample new particles
				var newBelief = [];
				for (var p = 0; p < NUM_PARTICLES; p++) {
					var rand = Math.random();
					var particleIndex = -1;
					while (rand >= 0) {
						particleIndex++;
						rand -= belief[particleIndex].probability;
					}
					newBelief[newBelief.length] = belief[particleIndex].location;
				}
				beliefs[i] = newBelief;
			}
		}

		var elapseTime = function (adversaries, robot) {
			for (var i = 0; i < adversaries.length; i++) {
				var belief = beliefs[i];
				for (var p = 0; p < belief.length; p++) {
					var motionModel = adversaries[i].motionModel(adversaries, robot, belief[p]);
					belief[p] = sampleFromDistribution(motionModel);
				}
			}
		}

		var addAgent = function (agent) {
			beliefs[beliefs.length] = uniformDistribution();
		}

		var removeAgent = function (i) {
			beliefs.splice(i, 1);
		}

		var convertToDistributions = function () {
			var distributions = [];
			for (var i = 0; i < beliefs.length; i++) {
				// Makes copy
				distributions[i] = beliefs[i].slice();
			}
			return distributions;
		}

		// Create return object
		this.setObservationModel = setObservationModel;
		this.observe = observe;
		this.elapseTime = elapseTime;
		this.addAgent = addAgent;
		this.removeAgent = removeAgent;
		this.convertToDistributions = convertToDistributions;
		return this;
	}

	var ParticleFilter = function (distributions, observationModel) {
		// Private variables
		var NUM_PARTICLES = 1000;
		var beliefs = [];
		var observationModelFunction = observationModel;

		// Create beliefs if given prior distributions
		if (distributions) {
			for (var i = 0; i < distributions.length; i++) {
				var singleDistribution = distributions[i];
				var singleBelief = [];
				for (var p = 0; p < NUM_PARTICLES; p++) {
					singleBelief[singleBelief.length] = sampleFromDistribution(singleDistribution);
				}
				beliefs[beliefs.length] = singleBelief;
			}
		}

		var setObservationModel = function (modelFunction) {
			observationModelFunction = modelFunction;
		}

		var observe = function (adversaries, robot) {
			var observations = observationModelFunction(adversaries, robot);
			for (var i = 0; i < beliefs.length; i++) {
				var observation = observations[i];
				var belief = beliefs[i];
				var totalWeight = 0;
				for (var p = 0; p < belief.length; p++) {
					var particle = belief[p];
					belief[p] = {
						location: particle,
						probability: getLocationProbability(particle, observation)
					};
					totalWeight += belief[p].probability;
				}
				// Normalize
				for (var p = 0; p < belief.length; p++) {
					belief[p].probability /= totalWeight;
				}

				// Sample new particles
				var newBelief = [];
				for (var p = 0; p < NUM_PARTICLES; p++) {
					var rand = Math.random();
					var particleIndex = -1;
					while (rand >= 0) {
						particleIndex++;
						rand -= belief[particleIndex].probability;
					}
					newBelief[newBelief.length] = belief[particleIndex].location;
				}
				beliefs[i] = newBelief;
			}
		}

		var elapseTime = function (adversaries, robot) {
			for (var i = 0; i < beliefs.length; i++) {
				var belief = beliefs[i];
				for (var p = 0; p < belief.length; p++) {
					var motionModel = adversaries[i].motionModel(adversaries, robot, belief[p]);
					belief[p] = sampleFromDistribution(motionModel);
				}
			}
		}

		var addAgent = function (agent) {
			var uniformDist = uniformDistribution();
			var newBelief = []
			for (var i = 0; i < NUM_PARTICLES; i++) {
				newBelief[i] = sampleFromDistribution(uniformDist);
			}
			beliefs[beliefs.length] = newBelief;
		}

		var removeAgent = function (i) {
			beliefs.splice(i, 1);
		}

		var convertToDistributions = function () {
			var distributions = [];
			for (var i = 0; i < beliefs.length; i++) {
				var distribution = []
				var belief = beliefs[i];
				for (var p = 0; p < belief.length; p++) {
					distribution[distribution.length] = {
						location: belief[p],
						probability: 1 / belief.length
					};
				}
				distributions[i] = distribution;
			}
			return distributions
		}

		// Create return object
		this.setObservationModel = setObservationModel;
		this.observe = observe;
		this.elapseTime = elapseTime;
		this.addAgent = addAgent;
		this.removeAgent = removeAgent;
		this.convertToDistributions = convertToDistributions;
		return this;
	}

	/****** World Representation *****/

	var adversaries = [];
	adversaries[0] = {
		location: new Location(),
		motionModel: randomMotionModel,
		color: {name: "cyan", value: "#00FFFF"}
	};
	var colors = [{name: "red", value: "#FF0000"}, {name: "green", value: "#00FF00"},
		{name: "blue", value: "#0000FF"}, {name: "yellow", value: "#FFFF00"}];
	var robot = {
		location: new Location(),
		motionModel: randomMotionModel,
		// beliefModel: new ExactInference(null, multipleAgentObservation)
		beliefModel: new ParticleFilter(null, multipleAgentObservation)
	};
	robot.beliefModel.addAgent(adversaries[0]);

	/****** Event Functions *****/

	var addAdversary = function () {
		// TODO: collect adversary information from doc
		// (Color, motion model, ...)
		var newAdversary = {
			location: new Location(),
			color: null,
			motionModel: randomMotionModel,
			apparentMotionModel: randomMotionModel
		};
	};

	var changeBeliefModel = function () {
		// TODO get new belief model
		// convert to that belief type
	}

	var changeMotionModel = function () {
		// TODO get new motion model
		// convert to that motion type
	}

	var changeObservationModel = function () {
		// TODO get new observation model
		// convert to that observation type
		// (Need to update belief model)
	}

	var updateWorldImage = function (distributions, adversaries) {
		canvas = document.getElementById("worldCanvas");
		context = canvas.getContext("2d");

		// Background
		context.fillStyle = "#000000";
		context.globalAlpha = 1;
		context.fillRect(0, 0, canvas.width, canvas.height);
		// context.fillStyle = "#FF0000";
		// context.globalAlpha = 0.5;
		// context.fillRect(20, 20, 20, 20);

		// Sizing
		squareSize = Math.min(canvas.width/WIDTH, canvas.height/HEIGHT);
		leftmost = (canvas.width - squareSize * WIDTH) / 2;
		topmost = (canvas.height - squareSize * HEIGHT) / 2;

		// Draw each agent
		for (var i = 0; i < adversaries.length; i++) {
			context.fillStyle = adversaries[i].color.value;
			for (var j = 0; j < distributions[i].length; j++) {
				var location = distributions[i][j];
				context.globalAlpha = location.probability;
				x = leftmost + squareSize * location.position.x;
				y = topmost + squareSize * location.position.y;
				context.fillRect(x, y, squareSize, squareSize);
			}
		}
	}


	var move = function(agent, adversaries, robot) {
		// BUG What the hell. Uncommented works, commented does not.
		var mod = agent.motionModel(adversaries, robot, agent.location);
		agent.location = sampleFromDistribution(mod);
		// agent.location = sampleFromDistribution(agent.motionModel(adversaries, robot, agent.location));
	}


	// Update loop
	var TIME = 1000
	interval = setInterval(function () {
		// observe
		robot.beliefModel.observe(adversaries, robot);
		updateWorldImage(robot.beliefModel.convertToDistributions(), adversaries);

		// move
		move(robot, adversaries, robot);
		for (var i = 0; i < adversaries.length; i++) {
			move(adversaries[i], adversaries, robot);
		}

		// elapseTime
		robot.beliefModel.elapseTime(adversaries, robot);
		setTimeout(function () {
			updateWorldImage(robot.beliefModel.convertToDistributions(), adversaries);
		}, TIME/2);
	}, TIME);
});