What we have:
- all sensory modules working independently with the .sh scripts
- all the readers receive the data. No particular processing is integrated for the moment.

What to do:
- create ball tracker
- create control loop
- change the readers with small pre processing accordingly

CHANGE THIS AS YOU DEEM FIT 

>[!NOTE]
> - The zynqGrabber.ini file is on the icub-ultrascale in ~/.local/share/yarp/cluster.
> - rgb /home/icub/yarp-device-ultrapython/ini/hiultra.ini

pkill -x ultrapythonui 2>/dev/null; yarp name unregister /xxx 2>/dev/null; (until yarp exists /xxx; do sleep 0.01; done; yarp connect /xxx /grabber/rpc fast_tcp) & ultrapythonui --remote /grabber



