# frame_dropper

This package has been included for easily replicating image flows, 
making it easier their configuration for image transmission over a data link. 

Note that the utilities assume 32FC1 format when publishing depth data.

Two utilities have been included:

## frame_dropper

This node gets an image from the topic "/in/rgb/image_raw" and republishes it rescaled and with frame dropping options. Parameters:

### ROS Parameters

* *in*: Uses resolvename to ask for the name of the input image flow
* *out*: Uses resolvename to ask for the name of the output image flow
* *frame_skip*: Number of frames to be received before publishing one image in the output flow
* *scale*: Scale (1 means no scaling)
* *publish_depth*: Also duplicates the depth image flow from /in/depth_registered/image_raw

### Published topics

* *out/rgb/image_raw* Image transport publisher with the output flow

### Subscribed topics

* *out/rgb/image_raw* Image transport subscriber to the input flow


## image_splitter

This node extends the functionality of frame_dropper by taking two different input image flows, that could be interchanged online by publishing in the "reverse" topic.

### ROS Parameters

* *in_1*: Uses resolvename to ask for the name of the first input image flow
* *out_1*: Uses resolvename to ask for the name of the first output image flow
* *in_2*: Uses resolvename to ask for the name of the second input image flow
* *out_2*: Uses resolvename to ask for the name of the second output image flow
* *frame_skip*: Number of frames to be received before publishing one image in the output flow
* *scale*: Scale (1 means no scaling)
* *publish_depth*: Also duplicates the depth image flow from /in/depth_registered/image_raw
* *use_depth*: If false, the depth image transport publisher is not instantiated
* *publish_all*: Publishes both flows

### Published topics

* *out/rgb/image_raw* Image transport publisher with the output flow

### Subscribed topics

* *out/rgb/image_raw* Image transport subscriber to the input flow
* */reverse* Out_1 takes the images from In_2 if /reverse is true. If false, normal flow
* */publish_depth* To active or deactivate depth publishing online
* */publish_all* Publishes both flows (in reverse order if publish depth is activated)
