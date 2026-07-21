https://www.yarp.it/latest/index.html
https://mesh-iit.github.io/documentation/
https://github.com/robotology/event-driven
https://robotology.github.io/robotology-documentation/doc/html/
https://github.com/robotology/event-driven/tree/v1.7/src/applications/gazeDemo

<application>
<name>vView </name>

<dependencies>
</dependencies>

<module>
    <name> zynqGrabber </name>
    <parameters>--left_off false --right_off false --sensitivity 65 </parameters>
    <node> icub-ultrascale </node>
</module>

<module>
    <name> vFramer </name>
    <parameters>--iso --height 480 --width 640 --src1 /zynqGrabber/right/AE:o --src2 /zynqGrabber/left/AE:o</parameters>
    <node>iiticublap267</node>
</module>

</application>