# Node Example

[ROS](http://ros.org) allows for creating nodes that communicate with each other.
It is very common to use C++ and Python to write these nodes.

This package contains example nodes written in C++ and Python that show minimal examples of using
some very basic but powerful features of ROS.
Those features include:

  * [parameter server](http://wiki.ros.org/Parameter%20Server)
  * [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigure/Tutorials)
  * [timers](http://wiki.ros.org/roscpp/Overview/Timers)
  * [custom messages](http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages)
  * classes with callback functions for
    [publishers and subscribers](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers)
  * [remap](http://wiki.ros.org/roslaunch/XML/remap) topic names

More ideas that are explored are deploying documentation using [GitHub Pages](https://pages.github.com/),
writing unit tests, and checking build status and code coverage.

## Description

There are several launch files included, the main one being `node_example.launch`.
This will start a talker and listener written in C++ and a talker and listener written in Python.
One GUI will open allowing you to see what messages are being recieved by the listeners and another GUI will allow
you to change the values sent from each talker.
Both listener nodes receive messages from both talkers, showing that the languages used to write the talkers and
listeners can be mixed.

## Usage

[Build a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) containing this repository.
A `node_example.rosinstall` file has been included for convenience with [`wstool`](http://wiki.ros.org/wstool).

To start all the nodes run

    roslaunch node_example node_example.launch

You should see two windows open: `rqt_reconfigure` and `rqt_console`.
They will look like the following screenshots.

  ![Reconfigure GUI](images/reconfigure.png)

  ![Console GUI](images/console.png)

At this point you can modify the strings or numbers in the reconfigure GUI and you should see those changes show
up in the console GUI.
There are `enable` parameters in each of the talker nodes so that the nodes can effectively be paused during runtime.
This is a nice feature that allows easily turning system components on and off during operation for whatever reason
(such as wanting to run multiple similar nodes side-by-side for comparison without using too many CPU/RAM resources,
only running certain nodes when some conditions are met, etc.).

## Branches

The `master` branch will try to keep up with the latest long-term support release version of ROS (currently Kinetic).
The `hydro-dev` branch was tested on ROS Hydro, Indigo, and Kinetic.
The `fuerte-dev` branch was tested on ROS Fuerte.

## Testing

During development there are large benefits to employing unit tests to verify that code changes do not break existing functionality.
This package contains unit tests for each of the C++ nodes.
The unit tests are run using the `*.test` files in the `test/` directory.
The `*.test` files start the node to be tested plus the unit test code.
The unit test code is written such that it publishes and subscribes to the topics that correspond to the interfaces of the node under test.
Callbacks are used to verify that the expected data is available on the specified topics.

There are several methods of running the unit tests.
Running the tests with continuous integration services for pull requests is a common method used to ensure pull requests can be safely merged.
A popular continuous integration provider for open source projects is [Travis CI](https://travis-ci.org).
The build and test results for this package can be found in the table at the top of this page.

Unit tests are not magic bullets.
The inputs to the nodes must take on enough values to verify that functions return valid values.
This will be different for each function and is not fully covered here.
Another aspect of unit tests is to ensure that all lines of code are exercised by unit tests, also referred to as code coverage.

A popular code coverage provider for open source projects is [codecov](https://codecov.io).
The code coverage results for this package can be found in the table at the top of this page.
This tool provides some measure of confidence that the existing unit tests will catch any issues, and that new changes are introduced with unit test code.

The configuration file for Travis is in this repository at [.travis.yml](.travis.yml).
That file contains build flags to ensure that unit tests run and that code coverage results can be calculated.
