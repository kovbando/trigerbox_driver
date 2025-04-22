# Triggerbox_driver
This package contains a node that publishes the Trigger ID from the camera sync hardware, each time it triggers.
The node can be configured from a launchfile. By default, it listens on port 5555 for UDP messages containing a simple ASCII coded integer, that will be the Trigger ID.
When a new ID arrives, it will be published as a simple integer (std_msgs/msg/Int32) to /trigerid by default, but the topic can be changed from launchfile.
code mostly generated with ChatGPT, but it is tested and working
