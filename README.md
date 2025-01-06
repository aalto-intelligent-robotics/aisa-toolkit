# AISA Toolkit

**AISA** is an extendable toolkit for situational awareness research in automated driving. 

The following functionalities are supported:
* Implementation of different types of conflict situations.
* Explanation of the driving situation by displaying information of different modalities (textual, visual).
* Initiating a takeover request to a human operator.
* Implementation of custom vehicle controllers.

(*AISA* stands for *AI-powered Situational Awareness*.)


## Setup

### Requirements

Running the simulation requires CARLA server, therefore we recommend following the [CARLA system requirements](https://github.com/carla-simulator/carla?tab=readme-ov-file#recommended-system):

* Intel i7 gen 9th - 11th / Intel i9 gen 9th - 11th / AMD ryzen 7 / AMD ryzen 9
* +32 GB RAM memory
* NVIDIA RTX 3070 / NVIDIA RTX 3080 / NVIDIA RTX 4090
* Ubuntu 20.04

The setup has been tested on Ubuntu 22.04, with a GPU NVIDIA RTX3080 with cuda drivers installed.
Less powerful GPUs should also be able to run the setup, but a GPU size of at least 8GB is recommended.

The simulation has also been tested on Windows, and instructions are provided below, but the recommendation is to use Ubuntu, as CARLA performs better on Ubuntu.

### Download CARLA 0.9.15

This code has been tested with the latest stable CARLA version - [CARLA 0.9.15](https://github.com/carla-simulator/carla/releases/tag/0.9.15).

From the link above, download the archive `CARLA_0.9.15.{tar.gz|zip}` to your computer and unarchive it. The path to the folder needs to be set as an environment variable for running the simulation (details in the section below).


### Setup the environment
Install [Conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html).

Create the Conda environment:

```bash
conda create -n aisa python=3.7
```

Create the Conda environment:

```bash
conda activate aisa
```

Install the necessary requirements:

```bash
pip install -r requirements.txt
```

## Running the Simulation

The simulation runs with the CARLA server started. 


### Running on Ubuntu

The **server and the toolkit will be run in separate terminal tabs**, following the instructions below.

#### Run CARLA

The first step in running the simulation is having a running CARLA server.
In the terminal, navigate to the CARLA installation directory and execute:

```bash
./CarlaUE4.sh 
```

In case with issues with the graphic adapter, try [this fix](https://github.com/carla-simulator/carla/issues/5970#issuecomment-1950604949):

```bash
./CarlaUE4.sh -ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter=2
```

#### Run the Simulation

First, open the file `run.sh` and **set the environment variable** `CARLA_HOME` to the path where the CARLA folder was saved.

Then, in a new terminal tab (different from the one in which CARLA is running), make sure that the Conda environment is activated:

```bash
conda activate aisa
```

And run the script:

```bash
./run.sh
```

The script `run.sh` executes the simulation main file `simulation.py`.


### Running on Windows

#### Starting the CARLA Server

To start the CARLA server, execute the file `CarlaUE4.exe`, located in your CARLA installation directory (should be something like `C:/Users/user/CARLA_0.9.15/WindowsNoEditor`).

#### Running the simulation

Set the path to the CARLA installation directory in the environment variable `CARLA_HOME`.

```bash
set CARLA_HOME=C:/Users/user/CARLA_0.9.15/WindowsNoEditor
```

In the command prompt, navigate to the toolkit folder, make sure the conda environment is activated.

When the CARLA server is running, execute the script `simulation.py` in the simulation directory.


```bash
conda activate aisa

cd simulation
python -m simulation --model lane_detection --conflict sensornoise
```

## Functionalities and Customization

### Lane Following and Takeover

Currently, the vehicle follows the lanes using a simple lane detection model. 
When the certainty of the predicted lanes is below certain value (0.6), a warning message is displayed.
When the certainty becomes below a specified takeover request threshold (0.3), the vehicle stops and the simulation switches to manual control.


### Confict Definitions

The scenarios to run are described in the file `scenarios/aisa_conflicts.xml`.

The following attributes can be given to a scenario:

* **name** - unique name of the scenario
* **town** - the CARLA map to load
* **npc_pedestrians** - number of pedestrians to be spawned on the map (*to be implemented*)
* **npc_vehicles** - number of pedestrians to be spawned on the map (*to be implemented*)
* **WeatherId** - ID of [CARLA weather presets](https://carla.readthedocs.io/en/stable/carla_settings/)
* **sensor_noise** - the amount of noise to add to the camera map image

A list of waypoints needs to be given, and a start and end waypoint need to be specified:

* **waypoint name="start"** - the initial waypoint where the ego vehicle will be spawned on the specified map
* **waypoint name="dest"** - destination waypoint. For the moment, only the route to it can be displayed, but navigating to it will be done in a later phase.

### Arguments

The following arguments are supported in the script `simulation.py`:

* `--model` - this is the model controlling the car. Currently, `lane_detection` is supported.

* `--conflict` - this is the name of a conflict from the conflict configuration file `scenarios/aisa_conflicts.xml`.

* `--audio` - if given, audio notification for takeover request is played.

* `--show_route` - if given, the route from he start to the end destination is displayed (*for now, the vehicle is not following this route, but this will be included in next versions*)


### Importing the Custom Map

To use the custom map, CARLA needs to be [built](https://carla.readthedocs.io/en/0.9.15/build_linux/) from source. The map can then be integrated within UE4. Place the .xodr and .fbx files in the **Import** folder according to the [instruction](https://carla.readthedocs.io/en/0.9.15/tuto_M_custom_map_overview/#importation).

The layout of the custom map can be customized. We have added the `HRI_custommap.blend` file for this purpose. This requires the [DrivingScenario](https://github.com/johschmitz/blender-driving-scenario-creator) add-on (at least v0.26.1). This was tested in Blender 2.93. Different textures can be applied to the roads of the custom map for the **Vanishing Lane Markings** conflict. We have only used textures from the CARLA assets. We recommend changing the segments `bdsc_export_road_clothoid_x` 40 to 44.


### Custom Controllers

Custom controllers can be implemented by extending the abstract class `ControllerModel`. Currently, a simple lane following controller is implemented.

A takeover request is initiated when the controller function `initiate_tor` returns True.
In this case, the takeover is communicated by text that appears on the screen and an optional audio signal.


## Code References

Control with lane detection is based on [Algorithms-for-Automated-Driving (AAD)](https://github.com/thomasfermi/Algorithms-for-Automated-Driving) and adapted to match the needs of the project.

Code from the CARLA simulator PythonAPI examples is used.


## Maintenance Plan

The toolkit requires a running CARLA server, but is not part of the CARLA repository. 
It will be available on GitHub as a standalone codebase upon acceptance and will be maintained by the authors.

### Python Version and Dependencies

CARLA Python API (client library) is packaged for specific Python versions, which are indicated in the .egg or .whl file names (e.g., cp37 for Python 3.7). The .egg file is located in the PythonAPI/carla/dist/ folder within your CARLA installation directory.
We use `Python 3.7` to match the `CARLA 0.9.15` client library. The versions of the required dependencies in`requirements.txt` are set to match Python 3.7.

In the near future, we plan to keep using CARLA 0.9.15, and if use of a newer CARLA version is necessary, the used Python and corresponding dependencies will be updated.

### Unreal Engine
We are following the migration from CARLA to Unreal Engine 5 (UE5). The associated CARLA version is still a dev branch, which is why we have implemented the project with version 0.9.15 in UE4. A completion of the UE5 version is targeted for 2025 and we will then upgrade the project as quickly as possible to avoid excessive divergences.

