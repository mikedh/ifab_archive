# ifab_archive

An archive for posterity of the SVN repository from [CMU's 2012 branch](https://www.scientificamerican.com/article/whos-the-boss-next-gen-factory-robots-could-call-shots/) of [DARPA's AVM/iFAB](https://en.wikipedia.org/wiki/Adaptive_Vehicle_Make) project.

This repository is BSD licensed as required by DARPA, which was very nice of them.

Note that this is extremely unlikely to be useful but might be interesting. Code is of the "make it work" variety and isn't exactly kernel- grade (well, maybe the kernel for an off- brand toaster oven). Some of the reusable parts were converted into independent projects, including:

- some very early and since re-written code for [trimesh](https://github.com/mikedh/trimesh) started in `trunk/dev/pycad`. For amusement check out the [original design intent doc.](ifab/trunk/dev/pycad/pycad.txt)

- the [open_abb](https://github.com/robotics/open_abb) robot driver hasn't been actively developed since but has been forked for use in a [number of](https://www.youtube.com/watch?v=pcp6kroGOVQ) [projects](https://www.youtube.com/watch?v=pcp6kroGOVQ)

- the design intent of AVM in general (3D model in, HMMWV out) formed the basis for the commercial [Kerfed Engine.](https://kerfed.com/technology)

### iFAB Demos

<iframe width="560" height="315" src="https://www.youtube.com/embed/aDzdxccX_RU" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
<iframe width="560" height="315" src="https://www.youtube.com/embed/2cyq28hoOQ4" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
