# PeopleFlow | Causal Effect Estimation of Robot Actions for Human-Aware Navigation

> This repository is a **fork of [lcastri/PeopleFlow](https://github.com/lcastri/PeopleFlow)**, extended for a
> Bachelor's thesis on causal effect estimation in human-aware robot navigation
> (University of Padova). The original simulator and all credit for it belong to its authors
> (see [Citation](#citation)). This README documents both the original framework and the
> additions made in this fork.

---

## This fork: what it adds

The base PeopleFlow simulator models context-sensitive human-robot spatial interaction. This
fork builds on it to answer a specific causal question:

> **Does a robot's interaction signal actually improve the outcome of navigating a congested
> corridor — and by how much?**

Rather than trusting the raw correlation between acting and succeeding (which, in this scenario,
is actively misleading), the effect of the robot's action is estimated from observational data
using **backdoor adjustment**, controlling for a deliberately injected confounder.

The contribution of this fork consists of:

- **A T-shaped corridor scenario** with a central bottleneck, populated by five pedestrian
  agents, where a TIAGo robot must traverse a potentially congested junction.
- **A controlled confounder mechanism**: static obstacles (suitcases) are spawned per episode
  and are visible to the robot but *not* to the pedestrian simulator, so that the confounder
  influences the robot's decision and the physical clearance, but never the pedestrians'
  behaviour.
- **A per-episode decision protocol**: the robot measures the bottleneck, probabilistically
  decides whether to signal, redirects nearby agents when it acts, and attempts the traversal
  unconditionally.
- **A full recording and extraction pipeline** that turns raw simulation logs into a clean
  causal dataset of one row per episode over six binary variables (Pi, A, Pe, S, T, O).
- **A causal analysis** estimating the effect of the action on the task outcome, including
  naive vs. backdoor-adjusted estimates, a confounder-variant analysis, and a causal discovery
  validation step.

The main empirical result: the naive estimate of the action's effect is negative (about
-0.21, a Simpson's paradox driven by the obstacle confounder), while every adjusted estimate is
small but positive (about +0.03 to +0.11). Reasoning causally, rather than correlationally, is
what separates "the action harms" from "the action helps".

**Thesis:** _Causal Effect Estimation of Robot Actions for Human-Aware Navigation_ (available on request).

---

## Fork-specific components

The following scripts and artefacts are specific to this fork and are not part of upstream
PeopleFlow. Paths are relative to the repository root.

| Component | Role |
| --- | --- |
| `<path>/TIAGo_plan.py` | Robot decision protocol: measures the bottleneck, decides and executes the HRI action, measures the outcome, publishes the DAG variables. Runs as a PetriNetPlans plan. |
| `<path>/obstacle_policy.py` | Samples the confounder O per episode and chooses obstacle positions (rejection sampling for minimum distance to agents/obstacles). |
| `<path>/DynamicObstacle.py` | Spawns/removes the obstacle models in Gazebo on request. |
| `<path>/PedsimBridge.py` | Assigns each pedestrian its next destination, and applies the hold/override parameters that redirect flagged agents to an evasion waypoint when the robot signals. |
| `<path>/record.py` | Records one rosbag per episode over the relevant topics. |
| `<path>/data_extractor.py` | Offline: converts each bag into a per-timestep time series (0.1 s) with all DAG variables and agent/robot positions. |
| `analysis/hrisim_causal_analysis.ipynb` | Loads the dataset, estimates the causal effect (naive, backdoor-adjusted, variant, discovery) and regenerates the figure used in the thesis. |
| `analysis/episodes_100_v1.csv` | The final dataset: 100 episodes, one row each. |

### Reproducing the dataset and analysis

1. Launch the simulation (`tstart`, see below) and let it collect episodes. One rosbag per
   episode is written by `record.py`.
2. Convert the bags into a time series:
   ```
   python3 data_extractor.py --bag_dir <bags> --output dataset_timeseries.csv
   ```
3. Aggregate the time series to one row per episode (last valid value of each DAG variable per
   episode) to obtain `episodes_100_v1.csv`.
4. Run `analysis/hrisim_causal_analysis.ipynb` to reproduce the estimates and the figure. See
   [`analysis/README.md`](analysis/README.md) for the dataset schema, the requirements and the
   list of values it reproduces.

---

# Original PeopleFlow

A Gazebo-based simulator designed to model context-sensitive human-robot spatial interactions
in shared workspaces. It features realistic human and robot trajectories influenced by
contextual factors such as time, environment layout, and robot state, and can simulate a large
number of agents. It involves a [TIAGo](https://pal-robotics.com/robots/tiago/) robot and
multiple pedestrians modelled using the [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros)
ROS library.

## Citation

If you find this repo useful for your research, please consider citing the following paper:

```
@article{CASTRI2026131246,
         title = {Causality-enhanced Decision-Making for Autonomous Mobile Robots in Dynamic Environments},
         journal = {Expert Systems with Applications},
         pages = {131246},
         year = {2026},
         issn = {0957-4174},
         doi = {https://doi.org/10.1016/j.eswa.2026.131246},
         url = {https://www.sciencedirect.com/science/article/pii/S0957417426001600},
         author = {Luca Castri and Gloria Beraldo and Nicola Bellotto},
         keywords = {Causal Discovery and Inference, Robot Autonomy, Human-Robot Spatial Interaction, Decision-Making},
}
```

## Features

* Customisable world
* Customisable people behaviours
* Customisable HRI scenario
* Customisable plans for the TIAGo robot
* Possibility to add context factors influencing human and TIAGo behaviours.

## How to use

### Build and Run with Docker Compose

After cloning the repository, navigate to the directory and use the `docker-compose` scripts to
manage the container.

1. **Build the Image:**
   ```
   cd /path/to/PeopleFlow
   ./docbuild.sh
   ```
2. **Run the Container:**
   ```
   ./docrun.sh
   ```

### Managing the Container

* **Access the Container Shell:**
  ```
  ./docshell.sh
  ```
* **Stop the Container:**
  ```
  ./docstop.sh
  ```

### Building ROS workspace

Once inside the container using `./docshell`, build the `ros_ws` workspace with:

```
catkin build
```

### Scenario setup and launch

Once inside the Docker container, view the `.tmule` file containing all simulator parameters:

```
roscd hrisim_tmule/tmule
cat hrisim_bringup.yaml
```

Parameters:

* TIAGO_TYPE - specifies the type of TIAGo robot;
* INIT_BATTERY - initial battery level of the robot. Default 100;
* STATIC_DURATION - battery duration (hours) when robot is idle. Default 5;
* DYNAMIC_DURATION - battery duration (hours) when robot is moving. Default 4;
* CHARGING_TIME - battery charging time (hours). Default 2;
* ABORT_TIME_THRESHOLD - Task completion deadline (seconds). Default 30 in this fork;
* WORLD - name of world and map to load. This fork uses `corridor`.
  If you want to add your own WORLD, include your .world file in hrisim_gazebo/worlds and your
  map in hrisim_gazebo/tiago_maps. Note that the map must have the same name as the .world file;
* SCENARIO - pedsim scenario to load. This fork uses `corridor`.
  If you want to add your own SCENARIO, include your .xml file in
  /pedsim_ros/pedsim_simulator/scenarios;
* ALLOW_TASK - if True, allows pedestrians to perform tasks when they reach their target position;
* MAX_TASKTIME - maximum task duration (seconds);
* GUI - if False, disables the Gazebo gui;
* MAX_STEP_SIZE - time (seconds) in the simulation to be simulated in one step;
* FORCE_OBSTACLE - social force model parameter ([Helbing et. al](https://arxiv.org/pdf/cond-mat/9805244));
* SIGMA_OBSTACLE - social force model parameter ([Helbing et. al](https://arxiv.org/pdf/cond-mat/9805244));
* FORCE_SOCIAL - social force model parameter ([Helbing et. al](https://arxiv.org/pdf/cond-mat/9805244));

To modify any of these parameters, edit the hrisim_bringup.yaml file:

```
roscd hrisim_tmule/tmule
nano hrisim_bringup.yaml
```

Once the tmule file is configured, start the simulator with:

```
tstart
```

to visualise the tmule session:

```
tshow
```

once inside the tmule, change panel with:

```
Ctrl+b
panel number [0-N]
```

and to stop it:

```
Ctrl+b
panel number 0
tstop
```

### Planning

ROS-Causal_HRISim includes [PetriNetPlans](https://github.com/francescodelduchetto/PetriNetPlans)
to define predefined plans for the TIAGo robot. A plan is a combination of actions and
conditions. Three folders are pre-created for plans, actions, and conditions:

* hrisim_plans
* hrisim_actions
* hrisim_conditions

For more details on how to define plans, actions, and conditions, visit the
[PetriNetPlans](https://github.com/francescodelduchetto/PetriNetPlans) GitHub repository.

## Recent changes

| Version | Changes |
| --- | --- |
| 1.1.0 | docker optimised |
| 1.0.0 | package released |
