---
name: Simulation Archive
about: Create a archive to keep track of the files
title: ''
labels: simulation archive
assignees: ''

---

<!-- We've collected some common issue solutions in https://taichi.readthedocs.io/en/stable/install.html#troubleshooting. Make sure you've check them out first. Hopefully they could address your problem. -->

**Describe the simulation goal**
A clear and concise description of why run this particular setup, ideally within 20 words.

**Build version**
Please post a ref to commit, on which you run like 972dd8bac4f831cb38355e0fbcaee1ade766e2fc

**Run command**
*Assumed running under `build/apps`, if not, please specify*

Please post a **minimal code** to reproduce. Ignore `bsub` or `nohup`

```bash
# sample code here
```


**Output folder**
Please specify a output folder


**Parameters.txt**
Please post the **full log** of the parameters here

```
$ ./my_app
#Global Parameter file
CurrentIterations:	0
Iterations:	500
ReportStep:	10
SizeX:	300
SizeY:	1000
tauFluid:	2.0
CDESolvers:	CDESolverD2Q5u	2.0
...
```

**Input Geometry**
Please use input geometry in `input_geo` for consistency

