# NATS Connector for ROS

Enables pub/sub messaging between two robots in different networks over NATS. The NATS Connector acts as a message forwarder between the local ROS network and the NATS network. Data flows from a publisher in ROS to a publisher in NATS which sends the messages to any subscriber connected to the NATS server, anywhere in the world. On the other end, a NATS subscriber forwards the data to a ROS subscriber which then uses that data from the original publisher.

## Usage

To use this package, add a single instance of the following node somewhere in your main launch-file and make sure to update the parameters according to your needs.

```xml
<node name="nats_connector" pkg="nats_ros_connector" type="nats_connector.py" output="screen">
    <param name="host" value="nats://host.docker.internal:4222" />
    <rosparam>
        publishers:
            - topic: talker1
              rate: 1
            - topic: talker2
              rate: 50
        subscribers:
            - topic: listener1
              type: std_msgs/string
            - topic: listener2
              type: std_msgs/string
    </rosparam>
</node>
```

-   In `host` you provide the address to your NATS server (in the example we are running ROS in docker in the NATS server on localhost)
-   In `publishers`, you provided a list of all the topics which you would like to publish from ROS to NATS. A `publisher` must contain:

    ```yaml
    topic # name of the ROS topic (use / for namespaces eg. foo/bar)
    rate # rate of publish over NATS. Defaults to 30 [Hz]. Can be different from the publish rate of the original ROS topic since the NATS connector stores the last published message in the ROS network when sending it to another NATS client.
    ```

    _`type` is not needed for publishers because the NATS connector forwards the message as serialized byte-string, using rospy.AnyMsg._

-   In `subscribers`, you provided a list of all the topics which you would like to subscribe to from NATS to ROS. A `subscriber` must contain:

    ```yaml
    topic # name of the ROS topic (use / for namespaces eg. foo/bar)
    type # Type of the topic
    ```

    _`rate` is not needed for subscribers because the NATS connector forwards a message from NATS to ROS upon arrival._

See [nats.launch](./launch/nats.launch) for an example.

## Installation

1. Clone this repository into the `src` folder in your ROS workspace.

    ```
    git clone https://github.com/aljanabim/nats-ros-connector
    ```

2. Install `nats-py`

    ```
    pip3 install nats-py==2.2.0
    ```

### Install Python Dependencies

Ensure pip is installed

```
sudo apt update
sudo apt install python3-pip
pip3 --version
```

then, install `nats-py`

```
pip3 install nats-py==2.2.0
```

## How it works

## Road Map

-   [x] Support Publishers
-   [x] Support Subscribers
-   [ ] Allow topic names that begin with forward-slash, ie. "/topic_name"
-   [ ] Support ROS Namespaces (via translation into NATS Subject token)
-   [ ] Support Services
-   [ ] Support Authenticated NATS Servers
