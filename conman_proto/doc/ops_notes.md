Orocos Script Notes
===================

### Connecting Things

#### Connectable

 * output port --> input port 
 * provided --> required

#### Connecting 

##### Connecing Ports in Provided Interfaces

Ports created on provided interfaces can be connected. 
Ports cannot be created on required interfaces.

```
connect("component.provided.outport","component.provided.inport",ConnPolicy())
connect("component.provided.subprovided.outport","component.provided.subprovided.inport",ConnPolicy())
```
