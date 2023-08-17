# Eye Tracking with Tobii Pro Grasses 3

## Tobii Pro Glasses 3 information
* API information [(here)](https://www.tobiipro.com/product-listing/tobii-pro-glasses3-api/)
* Field guide [(here)](https://connect.tobiipro.com/s/field-guide-glasses3)

<!-- ## Connecting to the recording unit/glasses
Our glasses model is the wired-only version. The glasses is first connected to the recording unit's hdmi port, then the recording unit can be connected to by ethernet cable. There are 2 ways to connect to the recording unit, through a router (**RECOMMENDED**) or directly to the computer.  -->

<!-- ### Connecting through a router (**RECOMMENDED**)
Connect the recording unit by ethernet cable to the router. The router will assign the recording unit an IPv4 address (log in to the router to check this), which can be used to access the web API guide and video streams. Connect the computer to the router (wired or wireless) and everything should be good to go. 

To access the web API guide, enter `http://<glasses-ip>/` in browser (works in Chrome) and replace `<glasses-ip>` with the assigned IPv4 address.

To view the video stream, open VLC, go to Media -> Open Network Stream, then enter `rtsp://<glasses-ip>:8554/live/all` and the stream from the glasses (no gaze overlay) will be shown.

### Connecting directly to computer by ethernet cable (Ubuntu)
When connecting directly to the recording unit, the recording unit will assign itself a link-local IPv6 address and the broadcast is only on IPv6 (not on IPv4). First, check if you can 
* ping the recording unit: `ping TG03B-080202023121.local` (you might have to wait a bit). 
    * `TG03B-080202023121.local` should resolve to an IPv6 link-local address (e.g. `fe80::76fe:48ff:fe6b:faea%eno1`). This IPv6 address is what you will use in code to connect to the Websocket.

If you are unable to ping the glasses, it is likely that mDNS is not configured for IPv6. Check that you have configured avahi-daemon for IPv6. Some helpful links:
* [How to enable mDNS for IPv6 on Ubuntu](https://unix.stackexchange.com/questions/586334/how-to-enable-mdns-for-ipv6-on-ubuntu-and-debian)

Now, if you are able to ping the glasses, open the Glasses 3 interactive API guide in a browser. In a browser, enter `http://TG03B-080202023121.local` and the interactive API guide should come up. If Chrome or Firefox don't work, try SeaMonkey. If the interactive API guide comes up, you are good to go!

Access to the rtsp stream by VLC or ffmpeg still do not work through the direct cabled connection. 


 -->

# Python Wrapper for Tobii Pro Glasses 3 API 
This module contains a python wrapper for interfacing with the glasses using websockets.

## Wrapper functionality 
id for each of the messages sent on the websocket uses the following convention:
| Path object   | ID   | 
| -----------   | ---- |
| system        | 1xxx |
| calibrate     | 2xxx | 
| network       | 3xxx |
| recorder      | 4xxx |
| recordings    | 5xxx |
| rudimentary   | 6xxx |
| settings      | 7xxx |
| upgrade       | 8xxx |

1st digit indicates which path object. 2nd digit indicates which child of the path object (if root, 0). 3rd digit indicates type of action (get, set, action, signal). 4th digit is just an id. These are wrapped in an enum which is defined in [`utilities.py`](utilities.py)

<!-- ## Development notes
Sending and receiving might need to be separated into 2 separate clients to handle websocket Signals on receiving side (where 1 sent message receives more than 1 response). -->


<!-- ## Mapping of user gaze position to screen position 
This mapping is done using a calibration marker attached to the upper left corner of the screen. The assumption is that the user is seated square facing the screen.  -->