#!/usr/bin/python

import rospy
import os.path
import rosbag
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import time

def check_new_files(files_new, files_old):
	"""
	Check if there is new file appear in the directory.
	If files_new contains files not exist in files_old, return filenames of those new files.
	"""
	result = []
	for x in files_new:
		if x not in files_old:
			result.append(x)
	return result


def process_single_bag(bag_name, rect_output_dir):
	print("Processing bag: {}".format(bag_name))
	msgs_all = {}
	for x in TOPIC_LIST:
		msgs_all[x] = []

	# read all msgs
	bag = rosbag.Bag(bag_name)
	for topic, msg, t in bag.read_messages(topics=TOPIC_LIST):
	    msgs_all[topic].append((msg, t))
	bag.close()

	files_old = os.listdir(TRACKING_DIR)
	# publish two sync message together
	for i in range(len(msgs_all[TOPIC_LIST[0]])):
		timestamp = rospy.get_rostime()
		for topic in TOPIC_LIST:
			# publish message
			msg_tmp = msgs_all[topic][i][0]
			# msg_tmp.header.stamp = timestamp
			pubs[topic].publish(msg_tmp)
			print(topic)
			print("publishing {}".format(msgs_all[topic][i][1]))
			print(msg_tmp.header.stamp)

		# wait for rectifier
		# print("Pressing any key to continue, e to exit") 
		# tmp = input("")
		# if (tmp is not None) and (tmp == "e"):
		# 	exit(0)
		
		# if new rectified image added, move it to target output directory
		files_to_process = []  			# files need to be moved to 
		rate = rospy.Rate(10) 
		while True:
			if rospy.is_shutdown():
				print("")
				exit(0)
			files_tmp = os.listdir(TRACKING_DIR)
			files_to_process = check_new_files(files_tmp, files_old)
			if len(files_to_process) == 1:
				break;
			elif len(files_to_process) > 1:
				print("[WARN] more than 1 new file appear in the tracking_dir!")
				exit(1)
			rate.sleep()
		files_old = files_tmp;


		# create output folder
		for x in files_to_process:
			os.system("mkdir -p " + os.path.join(rect_output_dir, str(i)))
			os.system("cp -r " + os.path.join(tracking_dir, x) + os.path.join(rect_output_dir, str(i)))
		# move the result file to target dist
	print("bag: {} procceed".format(bag_name))

def iterate_dirs(ROOT_DIR, OUTPUT_DIR, dir_path):
	current_dir = os.path.join(ROOT_DIR, dir_path)
	output_dir = os.path.join(OUTPUT_DIR, dir_path)

	for x in os.listdir(current_dir):
		if os.path.isfile(os.path.join(current_dir, x)):
			# print(os.path.join(current_dir, x))
			# looking for .bag files
			_, ext = os.path.splitext(x)
			if ext == ".bag":
				path_to_bag = os.path.join(current_dir, x)
				process_single_bag(path_to_bag, current_dir.replace(ROOT_DIR, OUTPUT_DIR))
				# print(os.path.join(current_dir, x))
				# print(os.path.join(current_dir, x).replace(ROOT_DIR, OUTPUT_DIR))

		elif os.path.isdir(os.path.join(current_dir, x)):
			iterate_dirs(ROOT_DIR, OUTPUT_DIR, os.path.join(current_dir, x))


# Parameters
ROOT_DIR = "/home/ernest/data/final_dataset/bag3"
OUTPUT_DIR = "/home/ernest/data/final_dataset_result/bag3"
TOPIC_LIST = ['/camera/color/image_raw', '/camera/depth/color/points']

# The output directory for planar rectifier. The directory will be tracked to copy rectifier output to OUTPUT_DIR
TRACKING_DIR = "/home/ernest/results/RectifiedImages"

os.system("mkdir -p " + OUTPUT_DIR)

# ============================ MAIN CODE ===============================
rospy.init_node('bag_player', anonymous=True)

pubs = {}
pubs['/camera/color/image_raw'] = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1000)
pubs['/camera/depth/color/points'] = rospy.Publisher('/camera/depth/color/points', PointCloud2, queue_size=1000)

# checking
for topic in TOPIC_LIST:
	assert topic in pubs.keys(), "topic: {} not in publisher!".format(topic)

iterate_dirs(ROOT_DIR, OUTPUT_DIR, "")
# bag = rosbag.Bag('/home/ernest/data/final_dataset/bag3/distance_1_25/1/dataset_1_0_1.bag')