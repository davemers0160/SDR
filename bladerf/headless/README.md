# BladeRF custom project generation

These instructions are for the bladeRF-micro (bladeRF 2.0).


## Adding your own platform revision (take from the bladeRF wiki and modified)
Working on bladerf-hosted.vhd is not a good idea if you plan to implement your own signal processing for the bladeRF, as you might want to go back to the original functionality without hassle. For this reason, the bladeRF codebase uses Quartus revisions to allow different FPGA design implementations to coexist.

To add your own revision to the bladeRF project, perform the following steps:

-- In hdl/fpga/platforms/bladerf-micro/vhdl/, create a new vhdl file for your top level architecture. It is possible to copy bladerf-hosted.vhd to bladerf-revision_name.vhd as a start.
-- In hdl/fpga/platforms/bladerf-micro/, create a new QIP file containing the necessary IP files, including your top level design file. Again, you can copy bladerf-hosted.qip to bladerf-revision_name.qip and just change the top level VHDL design file name to fit.
-- In hdl/fpga/platforms/bladerf-micro/build/bladerf.tcl, where all the revisions are created, add your revision by adding a line such as "make_revision revision_name" to make your revision known to the Quartus project.
-- In hdl/fpga/platforms/bladerf-micro/build/platform.conf, you need to add your revision to the list of allowed revisions. This can be accomplished by finding the "PLATFORM_REVISIONS" variable and adding your revision as an option there. Don't forget to change the help message in usage() to list your revision as well.

At this point, you should be able to build your revision using the build_bladerf.sh script, passing it your revision name as a parameter.


```
> bladerf-cli -v verbose -i




```