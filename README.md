

# Sensorimotor learning experiments with the Sphero robot (smp\_sphero)

05/2014 - 2017, Oswald Berthold

Python code from the Closed-loop sphero behaviour paper <sup><a id="fnr.1" class="footref" href="#fn.1">1</a></sup>.


## Preparation

This is built on top of the following stacks, make sure to have that
installed (tested only on Ubuntu) 

-   python-numpy, python-bluetooth, &#x2026; Use apt-get / distro mechanism
-   ROS base install, use apt-get
-   sphero\_ros driver from <sup><a id="fnr.2" class="footref" href="#fn.2">2</a></sup>, install via $ROSDISTRO\_workspace
    catkin\_make. See the ROS wiki how to set up a catkin workspace.

This is the launch sequence for talking to sphero with the ros driver:

    roscore    
    python src/sphero_ros/sphero_node/nodes/sphero.py

sphero.py accepts arguments such as update frequency sensorimotor loop
and now also a BT target address to connect directly to a given device

    python src/sphero_ros/sphero_node/nodes/sphero.py --freq 10

    python src/sphero_ros/sphero_node/nodes/sphero.py --freq 20 --target_addr sp:he:ro:sa:dd:re

Optionally, e.g. for sphero\_joystick.py, you need

    rosrun joy joy_node


## Scripts

When roscore and sphero.py are running and connected, try and and run
a few of the utils/test to check the basic communication.

<table border="2" cellspacing="0" cellpadding="6" rules="groups" frame="hsides">


<colgroup>
<col  class="org-left" />

<col  class="org-left" />
</colgroup>
<thead>
<tr>
<th scope="col" class="org-left">**Utils**</th>
<th scope="col" class="org-left">&#xa0;</th>
</tr>
</thead>

<tbody>
<tr>
<td class="org-left">bluerssi.py</td>
<td class="org-left">Log BT rssi values</td>
</tr>


<tr>
<td class="org-left">sphero\_colors.py</td>
<td class="org-left">Basic color actuation with ros</td>
</tr>


<tr>
<td class="org-left">sphero\_joystick.py</td>
<td class="org-left">Control sphero via ros joy\_node</td>
</tr>


<tr>
<td class="org-left">sphero\_raw.py</td>
<td class="org-left">Minimal test of raw communication protocol</td>
</tr>


<tr>
<td class="org-left">sphero\_simple\_openloop.py</td>
<td class="org-left">Simple open-loop command test</td>
</tr>


<tr>
<td class="org-left">sphero\_test.py</td>
<td class="org-left">Another minimal test</td>
</tr>
</tbody>
</table>


### Work in progress

When that works, you could try the learning experiments, just to
disentangle possible sources of errors. **<span class="timestamp-wrapper"><span class="timestamp">&lt;2017-02-28 Di&gt; </span></span> - This
doesn't work yet**: It needs more cleaning work on my side, hope to be
done soon.

<table border="2" cellspacing="0" cellpadding="6" rules="groups" frame="hsides">


<colgroup>
<col  class="org-left" />

<col  class="org-left" />
</colgroup>
<thead>
<tr>
<th scope="col" class="org-left">**Experiments**</th>
<th scope="col" class="org-left">&#xa0;</th>
</tr>
</thead>

<tbody>
<tr>
<td class="org-left">sphero\_res\_learner\_1D.py</td>
<td class="org-left">&#xa0;</td>
</tr>


<tr>
<td class="org-left">sphero\_res\_learner\_2D\_polar.py</td>
<td class="org-left">&#xa0;</td>
</tr>


<tr>
<td class="org-left">sphero\_res\_learner\_2D.py</td>
<td class="org-left">&#xa0;</td>
</tr>


<tr>
<td class="org-left">sphero\_res\_learner\_1D\_analyze.py</td>
<td class="org-left">&#xa0;</td>
</tr>


<tr>
<td class="org-left">sphero\_data\_recorder.py</td>
<td class="org-left">&#xa0;</td>
</tr>
</tbody>
</table>


# Footnotes

<sup><a id="fn.1" href="#fnr.1">1</a></sup> Berthold and Hafner, 2015, Closed-loop acquisition of behaviour on the Sphero robot, <https://mitpress.mit.edu/sites/default/files/titles/content/ecal2015/ch084.html>

<sup><a id="fn.2" href="#fnr.2">2</a></sup> <https://github.com/x75/sphero_ros>
