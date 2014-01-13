Conman Orocos Scripting Reference
=================================


There are two ways to add a component to a conman scheme. Either by adding the component to the scheme as a peer, or by setting its slave activity explicitly.

```
addPeer("scheme","my_block");
scheme.addBlock("my_block");
```

```
setMasterSlaveActivity("scheme","my_block");
scheme.addBlock("my_block");
```

Then, you want to set the minimum desired period for the component:

```
my_block.conman_hook.setDesiredMinPeriod(0.0015);
```
