# planar_rect_homography
The implementation of planar rectification from RGB-D information.
---
## Paper
Our paper proposed an improved method for the detection of planar objects, which rectifies images with geometric information to compensate for the perspective distortion before feeding it to the CNN detector module, typically a CNN-based detector like YOLO or MASK RCNN. Besides, we also release an [*RGB-D dataset*](), which is the first public-available hazmat sign detection dataset with RGB-D sensors. 

The paper describing the method was accepted for publication at SSRR2020 and should be published soon after the conference. 

## Usage

### Yolo Training

### Rectification
```bash
# start the rectification node
roslaunch planar_rect_homography_pkg planar_rectifier_exp.launch
```
```bash
# set camera_info and tf tree
rosbag play info.bag

```
```bash
# auto-iterate through target directory and play messages one by one
# it will monitor the rectifier status flag (ros message)
roslaunch hazmat_bag_tools play_bags
```
This will prepare the test dataset (for yolo in the next step) in the folder ```results```

### YOLO testing
Use following command to run yolo inference and save results to ```playground/output.txt```
```bash
./darknet detector test ~/workspace/mars_hazmat_detection/experiments/cfgs/exp9/hazmat_valid_bag3.data ~/workspace/mars_hazmat_detection/experiments/cfgs/exp9/yolov3-tiny-13.cfg checkpoints_9/yolov3-tiny-13_last.weights -dont_show -ext_output  </home/ernest/playground/input.txt >/home/ernest/playground/output.txt
```
### Evalutation (with NMS)
In evaluation we need the output.txt file generated in the previous step.  
Code of evaluation is inside `evaluation.ipynb`, remember to change the paths when using. You will find the detection result inside the `nms_img` folder.
