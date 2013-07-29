Conman
======

## Introduction

*Conman* is a controller and state estimator manager built on top of the
[Oroocos Toolchain](http://www.orocos.org) and designed to create a common
platform for sharing robot controllers and state estimators.

### Audience

This tool is meant for people who wish to do real-time robot state estimation
and control within the Orocos framework. While this is a ROS-independent
framework, tools for ROS integration are provided in the **conman_ros** package.

### Scope

*Conman* not trying to solve the world. *Conman* is designed with the following goals:
* Provide a common interface for running Orocos components for robot state estimation and control.
* Handle the computational scheduling of these blocks.
* Provide a special component, not a special framework.

### Approach

*Conman* is primarily concerned with two features of control schemes:
* Correctly serialized computation of estimation and control blocks
* Defining a set of standard Orocos/RTT interfaces for communicating between
  such blocks.

#### Serialized RTT Component Computation 

A *Conman* "scheme" is a pair of directed acyclic computational graphs meant to
be computed topologically in realtime: the **estimation graph** and the **control
graph**. These two graphs may share vertices (RTT components), but are
distinguished by their arcs (relevant RTT ports). 

Each scheme update involves the following stages:

1. Read from all hardware (sensors)
2. Compute **estimation graph**
3. Compute **control graph**
4. Write to all hardware (actuators)

#### Common RTT Port Interfaces

We do not want to define _yet another_ datatype or message specification.
Instead, we want to take existing datatypes (Eigen, KDL, ROS, etc) and annotate
them with relevant metadata needed to make sense of them. Essentially, we want
any given port to be self-describing. 

For example, a port in joint-space should provide both the number of degrees of
freedom of the joint group, as well as the ordered list of joint names, and a
port in cartesian-space should also provide the name of its origin frame.

### Prior Work

There has long been a desire to have a common platform for building and sharing
robot control "blocks." Orocos RTT, itself, aims to be such a platform, however,
with great power comes great flexibility. This means that the many ways that
Orocos RTT provides to exchange information between components and different
ways to schedule components makes it hard for developers to come to a consensus
on how data should move around the system.

This is the primary motivation of the [Rock](http://rock-robotics.org)
framework, but this framwork aims to solve far more than this problem. One
feature of Rock, however, is a notion of component dependency management which
can allow the system to reason about which components are needed for downstream
componentns.

In [April 2012](http://www.orocos.org/forum/rtt/rtt-dev/ideas-about-formalizing-and-tooling-composition-primitives),
Piotr Trojanek outlined a desire for "best practices" for RTT component
composition on the RTT Developers list. While not specific to constructing a
controller manager, there are several opinions on how to model more general
systems which are relevant to this project and prior work. The contributors in
this discussion made the following points:

* **Piotr Trojanek**
  * A lot of what's needed to formalize/standardize composition might already be
    in Rock, but Rock adds even more ways to specify component interactions.
  * Some of Rock's design paradigms can be considered analogous to
    object-oriented design (templates <-> specializations, inheritance <->
    refinement, etc). This is useful because these analogies can be used to
    communicate the semantics of some of the composition primitives.
  * SIMULINK-like composition is also desirable, with graphical interfaces.
  * It's unclear how sequencer/execuation frameworks (TDL/PLEXIL) fit into
    component-based design, but DSMLs can be useful.
  * There is a lot of nomenclature which needs to be better defined. Some of these have been defined by Szyperski, but also [OMG](http://www.omg.org/spec/RTC/1.0/) has formalized a lot of compositional design patterns.
  * DAGs and Polytrees yeild obvious serialization, so they are useful
    constraints for modelling.
  * At the time of writing, it is not possible to use a single cosntraint
    language across the entire toolchain.
  * OO-based diagrams can't capture everything needed for component-based
    design.

* **Herman Bruyninckx**
  * Essential difference between component-based and object-oriented design [link (PDF)](http://atlanmod.emn.fr/www/papers/OnTheUnificationPowerOfModels.pdf) and this is clarified further by [link (PDF)](https://oxygen.informatik.tu-cottbus.de/IT/Research/AssmannZW06.pdf).
  * Operations are needed on the modelling level.
  * Making Rock's meta-model explicit could be useful, even as a learning
    experience.
  * "Port-renaming" is an important compositional tool.
  * There is no apparent reason to limit the topology of the computational
    structures to DAGs or Polytrees; the "serialization" can be done on graphs
    via (possibly multiple alternative) scheduling data structures at the
    composition level. Each of these will superimpose a DAG/Polytree on top of
    the graph topology.
  * Schedulers are an important facet of the computational graph model.
  * One of the big lack of Piotr's originally proposed graphical modelling
    tools is a set of formal constraint languages.
  * Execution frameworks don't really fit into the "component model" but they
    still need to be modelled.
  * The MDE M3-M0 modeling paradigm was incorporated into the BRICS Component Model. [link (PDF)](http://www.clawar.org/downloads/Robot_Modularity/presentations/4.%20BRICS%20Component%20Model-Bruyninckx-20130509.pdf) [link (PDF)](http://lucagherardi.it/wp-content/papercite-data/pdf/brics2013component.pdf).

* **Geoffery Biggs**
   * OpenRTM supports forming and breaking compositions at runtime without
     affecting the lifetime of the components

* **Nico Hochgeschwnder**
   * It's not so bad to have more than one constraint language, it depends on
     what we want to do with the constraints.

* **Sylvain Joyeux**
  * Operatins are not used or needed on the modelling level in Rock, so they are
    not promoted.
  * As in OpenRTM, compositions can change at rutime without affecting component
    lifecycle.

In [January 2013](http://www.orocos.org/forum/orocos/orocos-users/joint-controller-manager),
Adolfo Rodríguez Tsouroukdissian initiated a more recent discussion, inspired by
the PR2 controller manager on the Orocos Users list. The contributors in this
discussion made the following pouints:

* **Adolfo Rodríguez Tsouroukdissian**
  * This proposal has the same goals as the [ros\_control framework](http://www.github.com/ros-controls/ros_control)
  * Provided an example data flow diagram [link (PDF)](http://www.orocos.org/files/joint_controller_manager.pdf)
  * Proposed a joint-level controller manager, motivated by the design of the
    PR2 controller manager
  * It's important to not require a single controller provide the full pipeline
    of computation from ROS command interfaces to hardware joint-level
    interfaces
  * The time reference should be external to the computational blocks

* **Piotr Trojanek**
* **Sylvain Joyeux**
* **Herman Bruyninckx**
 

