Conman
======

[![Build Status](https://travis-ci.org/jbohren/conman.png?branch=master)](https://travis-ci.org/jbohren/conman)

## Introduction

*Conman* is a controller and state estimator manager built on top of the
[Oroocos Toolchain](http://www.orocos.org) and designed to create a common
platform for sharing robot controllers and state estimators. Conman enforces
additional constraints on when and how components can be running in order to
prevent errors and improve robustness when switching components in and out
of the control loop at runtime.

### Audience

This tool is meant for robotics researchers who wish to do real-time robot
state estimation and control within the Orocos real-time toolkit (RTT). While
this is a ROS-independent framework, tools for ROS integration are provided in
the **conman_ros** package. 

### Scope

*Conman* not trying to solve the world. *Conman* is just trying to get rich
quick. As such, it is designed with the following goals:

* Provide a common interface for running Orocos components for robot state
  estimation and control.
* Provide a more-constrained modeling paradigm for robot state estimation and
  control.
* Determine the computational scheduling of these blocks from their data flow
  connections and desired execution rates.
* Enable external manipulation of the running set of estimators and controllers
  so that single blocks and groups of blocks can be enabled and disabled at
  runtime.
* Provide a special component, not a special framework to perform these roles.

### Approach

#### Schemes: Serialized Control, Hardware, and Estimation Model Execution 

A *Conman* "scheme" models the data flow of and execution constraints on a set
of RTT components connected by data flow ports. The scheme is an RTT component,
itself, and when executed at runtime, it serially executes a subset of its peers
according to their input and output data flow ports. 

The execution schedule guarantees that all inputs to a given component are
generated before said component, and each component is executed in a single
update of the scheme. This schedule corresponds to a topological sort of the
data flow graph. As such, cycles in data flow need to be explicitly broken
through "latching." Latching the data connections from component *A* to
component *B* means that the data that component *B* reads at cycle *(k)* will
be the data that component *A* wrote at cycle *(k-1)*.

#### Scheme Internals

Internally, the scheme is represented by three graphs:

1. **Data Flow Graph** (DFG): An exact directed graph model of the data flow
   between the scheme members. Each vertex represents an RTT task, and each
   arc represets a set of data flow connections between two tasks.
   This graph may be cyclic.
2. **Execution Scheduling Graph** (ESG): The DFG with "latched" edges removed.
   This graph is always acyclic, and is used to compute the execution order of
   the scheme members via standard topological sort.
3. **Runtime Conflict Graph** (RCG): An undirected graph of members where
   adjacent members cannot be run simultaneously due to some resource conflict.

#### Scheme Construction

Components are added to and removed from a scheme procedurally. Each time a
component is added, the scheme regnerates its model of the data flow and
conflict relationships between all the scheme members.

Currently, the scheme topology can only be changed when it is not in the *Running*
state. In the future, the scheme model will be double-buffered so that a copy of
it can be modified at runtime and then swapped in real-time with the active one.

#### Edge Latching

If the addition of a component to the scheme adds a cycle to the data flow
graph, one of the connections in the cycle must be latched before the scheme can
be executed in an unambiguous order. Latches are also set procedurally.

#### Component Grouping

Components in the scheme can be grouped together under alphanumeric labes (and
groups can contain other groups). Groups are useful for starting and stopping
multiple scheme components simultaneously at runtime. The manipulation of groups
does not change the graph topology.

#### Scheme Orchestration at Runtime

Components can be started and stopped synchonously with the execution of the
entire scheme at runtime. If numerous components are specified, they are started
in topological order, and stopped in reverse-topological order.

### Designing Components for Use in Conman

Conman imposes a few constraints on the design of RTT components. For components
to be used with Conman, they need the following properties:

* The component should be compatible with the `SlaveActivity` execution pattern.
  Each component added to a Scheme is assigned a `SlaveActivity` which enables
  it to be executed in the Scheme's thread.
* The component's `startHook()`, `updateHook()`, and `stopHook()` should be
  realtime-safe and not block for long durations. Components should either
  compute their results with bounded latency, or coordinate without locking with
  a separate activity. 
* Data ports should only be written to and read from in a component's
  `startHook()` and `updateHook()`. The assumption of the sched

#### Running Components at Different Rates

The *Conman* scheme will run at a fixed rate (the rate at which you want to
control the fastest hardware in the scheme), but you may want to have components
which run slower than that. If this is the case, then they can still run at
integral multiples of the loop rate.

Each member of the scheme has a minimum execution period associated with it. By
default, this minimum execution period is 0.0 seconds. A minimum execution
period of 0.0 seconds means that the block will be executed as fast as the
scheme, itself.

#### Common RTT Port Interfaces

***IN PROGRESS***

We do not want to define _yet another_ datatype or message specification.
Instead, we want to take existing datatypes (Eigen, KDL, ROS, etc) and annotate
them with relevant metadata needed to make sense of them. Essentially, we want
any given port to be self-describing. 

For example, a port in joint-space should provide both the number of degrees of
freedom of the joint group, as well as the ordered list of joint names, and a
port in cartesian-space should also provide the name of its origin frame.

##### Joint-Space Quantity
##### Cartesian-Space Quantity

## Tutorials 

### Running a Conman Scheme

**TBD**

### Writing a State Estimation Block

**TBD**

### Managing a Scheme At Runtime

**TBD**

### Running Conman In Gazebo

**TBD**

### Loading a ros\_control-based Controller Into Conman

**TBD**

### Loading a ros\_control-based Hardware Interface Into Conman

**TBD**

## Motivating Use Cases

### Teaching: My First Controller™ ###

**TBD**

### Controller Switching at Runtime

**TBD**

### Automatic State Estimation Switching at Runtime

**TBD**

### Visual Feedback for Robotic Manipulator Control

**TBD**

## Future Work

### Running Multiple Schemes

Since *Conman* schemes do not _own_ the RTT components involved
in them, it's possible to run multiple schemes either in parallel or serialized.
Of course there's nothing to worry about if the two schemes do not interact, but
if they do, you can still run them in parallel...


