# pip install pyrealsense2
# https://dev.intelrealsense.com/docs/python2
# https://www.intelrealsense.com/wp-content/uploads/2019/09/Intel_RealSense_Tracking_Camera_Datasheet_Rev004_release.pdf page 28

import pyrealsense2 as rs
import numpy as np

from scipy.spatial.transform import Rotation as R
import time

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
# cfg.enable_stream(rs.stream.accel)
cfg.enable_stream(rs.stream.gyro)

profile = pipe.start(cfg)
# profile = pipe.start()

v_rot = np.asarray([0, 0, 0])
up_vect = np.asarray([0, 0, 1])
r2 = R.from_matrix([[0, 0, -1],
                   [-1, 0, 0],
                   [0, 1, 0]])
r0 = r2.inv()
rtest = R.from_matrix([[0, -1, 0],
                   [0, 0, 1],
                   [-1, 0, 0]]).inv()
				   
uz = np.asarray([0, 0, 1])

all_up = []
all_v = []
all_t = []

first_wait = True

def read_imu (debug=False):
	frames = pipe.wait_for_frames(100 if not first_wait else 5000)
	first_wait = False
	for f in frames:
		# print(f.profile.stream_type())
		if f.profile.stream_type() == rs.stream.gyro:
			rot_data = f.as_motion_frame().get_motion_data()
			v_rot = rtest.apply([-rot_data.x, rot_data.y, -rot_data.z])
			
		if f.profile.stream_type() == rs.stream.pose:
			data = f.as_pose_frame().get_pose_data()
			r1 = R.from_quat([data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]).inv()
			up_vect = r2.apply(r1.apply(r0.apply(uz)))

if __name__ == "__main__":
	try:
		# for i in range(0, 10):
		start = time.time()
		while time.time() < start + 60:
			frames = pipe.wait_for_frames(100 if not first_wait else 5000)
			first_wait = False
			for f in frames:
				# print(f.profile.stream_type())
				if f.profile.stream_type() == rs.stream.gyro:
					rot_data = f.as_motion_frame().get_motion_data()
					v_rot = rtest.apply([-rot_data.x, rot_data.y, -rot_data.z])
					
				if f.profile.stream_type() == rs.stream.pose:
					data = f.as_pose_frame().get_pose_data()
					r1 = R.from_quat([data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]).inv()
					up_vect = r2.apply(r1.apply(r0.apply(uz)))
			print(v_rot)
			print(up_vect)
			print()
			"""
			pose = frames.get_pose_frame()
			if pose:
				if False:
					# Print some of the pose data to the terminal
					data = pose.get_pose_data()
					print("Frame #{}".format(pose.frame_number))
					print("acceleration: {}".format(data.acceleration))
					print("angular_acceleration: {}".format(data.angular_acceleration))
					print("angular_velocity: {}".format(data.angular_velocity))
					print("mapper_confidence: {}".format(data.mapper_confidence))
					print("rotation: {}".format(data.rotation))
					print("tracker_confidence: {}".format(data.tracker_confidence))
					print("translation: {}".format(data.translation))
					print("velocity: {}".format(data.velocity))
					print()
					r1 = R.from_quat([data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]).inv()
					print(r0.apply(uz))
					print(r1.apply(r0.apply(uz)))
					print(r2.apply(r1.apply(r0.apply(uz))))
				if True:
					data = pose.get_pose_data()
					r1 = R.from_quat([data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]).inv()
					up_vect = r2.apply(r1.apply(r0.apply(uz)))
					v_rot = rtest.apply([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
					# print(up_vect, "\t", v_rot)
					print(data.angular_velocity)
					print(v_rot)
					print()
				"""
			time.sleep(0.3)
	finally:
		pipe.stop()
