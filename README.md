# 2dlaser_people_detect
this is a ros package classifing people and other object using 2d lidar scan data.

the procedure as below:
1. cluster scan data
2. extact features
3. training a random forest model



this is successfully build under ros indigo with ubuntu 14.04
how to use:
1. prepare your laser scan bags, people/others/mix bags
2. edit launch/train_body_detector.launch, the path of bags and other params
3. run  roslaunch laser_people_detect train_body_detector.launch
