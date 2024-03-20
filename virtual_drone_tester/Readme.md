# Basic Instruction to Work with Webot

## Downloading Webot

Download from [Cyberbotics](https://cyberbotics.com/). Follow all the instructions. The default installation setting is perfectly fine for our application. 

## Programming in Webot

We will be using C++ in Webot - it comes with MinGW compiler, but that is only available with the IDE in Webot. For coding in a different IDE (ex. VS Code), MinGW should be installed locally as well. For VS Code, install [MinGW](https://code.visualstudio.com/docs/cpp/config-mingw) and make sure that both gcc and g++ are installed and added to path. 

[Documentation](https://cyberbotics.com/doc/guide/cpp-java-python)


[Controller Architecture](https://cyberbotics.com/doc/guide/controller-programming)


[Reference Manual](https://cyberbotics.com/doc/reference/index)

## General File Structures

Webot requires the presence of the following folders, contained under a singular directory: libraries, controllers, plugins, protos, and worlds, even though most of them will remain empty for us. The main Webot file has a .wbt extension, and it is kept under the worlds directory. The .stl files imports the geometrical mesh of our drone - it is mostly for cosmetic reasons, unless we import our own physics engine (probably through the fluids node, and a custom bounding object) once we are at the stage where we want to test the controller with realistic aerodynamic properties. 

### Loading the CAD file

Our .stl files are too large for github to handle, so it must be manually inserted into the directory, under "worlds". It should automatically load when you open the Webot file. If it does not, do into the directory /Robot "tester drone"/children/Shape/Mesh/url and select the .stl file manually.

[comment]: <> (Upload the .stl files to google drive, delete this line once you have done so)

## Creating a new controller

Create a new folder under the controller folder. Then, under /Robot "tester drone"/controller, select the correct folder name. In said folder, if there is only a singular .cpp file, Webot will automatically load the .cpp file under edit mode.

Afterwards constructing the .cpp file, compile it with g++ in the MinGW UCRT64 terminal:

```cd directory```

```g++ filename.cpp```

If an error pops up regarding unable to locate the .hpp files

```export CPLUS_INCLUDE_PATH C:/"Program Files"/Webots/include/controller/cpp/webots```

Then, compile again.

or, alternatively, compile in Webot, which automatically remembers the default paths for all header files.

# Model Considerations

CG: [0 0 .015] in reference to the vehicle center in Webot's reference frame (center of the fuselage is .169m above the global z, and at the local origin of the QRBP)
Orientation/Reference Frame in Control Algorithms: conforms to Kristoff's model, where nose up is positive x, pitch is along the y-axis
Moments of inertia: Scaled up, then tweeked slightly from the CRC-3 Model (estimation, modeling for a QRBP, **conforming to the Webot reference frame**)
Jxx (pitch) = .014 kg m^2
Jyy (roll) = .012 kg m^2
Jzz (yaw) = .020 kg m^2

Products of inertia: assumed to be ~0 due to the lack of asymetry in the design
Mass: 1.8kg or ~4lb

Aerodynamic forces are not yet considered, but there should be 2 major considerations:
Rotor wake and subsequent lift generation via the wings
Fuselage drag (considering that CD ~ .015, somewhat insignificant at this stage)
