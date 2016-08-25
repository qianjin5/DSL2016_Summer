import mysql.connector
from mysql.connector import errorcode
import numpy as np
import json
import math
import matplotlib.pyplot as plt

'''
Usage: See __main__ as example
Contact: zining.zhu@mail.utoronto.ca
'''


class DesTrajGetter:
	def __init__(self):
		from demo_config import configInfo
		self.config = configInfo


		#self.traj_num = len(self.get_all_tracks())


	def get_all_tracks(self):
		all_tracks = []
		cnx = mysql.connector.connect(**self.config)
		query = ("select traj_id, trajname, destraj, orig_range from DSLPath_tbl")
		cursor = cnx.cursor()
		cursor.execute(query)
		for (traj_id, trajname, destraj, orig_range) in cursor:
			traj = DesTraj(traj_id, trajname, destraj, orig_range)
			all_tracks.append(traj)
		cursor.close()
		cnx.close()
		return all_tracks

	def get_newest_traj(self):
		cnx = mysql.connector.connect(**self.config)

		query = ("select traj_id, trajname, destraj, orig_range from DSLPath_tbl where traj_id in (select MAX(traj_id) from DSLPath_tbl)")
		cursor = cnx.cursor()
		cursor.execute(query)
		all_trajs = []
		for (traj_id, trajname, destraj, orig_range) in cursor:
			traj = DesTraj(traj_id, trajname, str(destraj), orig_range)
			all_trajs.append(traj)

			from demo_config import trajinfopath, phys_range
			infodict = {"traj_name": trajname, "traj_id": traj_id, "avgerr": None,
				"orig_range": str(orig_range)}

			f = open(trajinfopath, "w+")
			f.write(json.dumps(infodict))
			f.close()
			f = open(trajinfopath, "r")
			#print "desired_trajectory.py written infodict to %s: %s" %(trajinfopath, f.read())
			f.close()

		assert len(all_trajs) == 1

		cursor.close()
		cnx.close()
		res = all_trajs[0]


		return res

	def get_traj_by_name(self, name):
		''' Only gets the newest trajectory with trajname name
		'''
		cnx = mysql.connector.connect(**self.config)
		cursor = cnx.cursor()
		query1 = ("select traj_id, trajname from DSLPath_tbl where trajname='%s';" %str(name))
		cursor.execute(query1)
		ids = []
		for (i, trajname) in cursor:
			ids.append(i)
		query2 = ('''select traj_id, trajname, destraj, orig_range from DSLPath_tbl
			where traj_id=%d;''' %max(ids))
		cursor.execute(query2)
		all_trajs = []
		for (traj_id, trajname, destraj, orig_range) in cursor:
			traj = DesTraj(traj_id, trajname, str(destraj), str(orig_range))
			all_trajs.append(traj)

			from demo_config import trajinfopath, phys_range
			infodict = {"traj_name": trajname, "traj_id": traj_id, "avgerr": None,
				'orig_range': str(orig_range)}

			f = open(trajinfopath, "w+")
			f.write(json.dumps(infodict))
			f.close()

		cursor.close()
		cnx.close()
		res = all_trajs[0]


		return res


	def num_traj(self):
		return len(self.get_all_tracks())



	def get_default_traj(self):
		default_traj_dict = {}
		r = 2.0
		for t in np.linspace(0, 100, 11):
			w = 2 * np.pi / 100
			theta = t * w
			row = {
				'x': r * np.cos(theta),
				'y': r * np.sin(theta),
				'vx': -r * w * np.sin(theta),
				'vy': r * w * np.cos(theta)
				}
			default_traj_dict[t] = row
		return default_traj_dict


################
# Helper Class #
################
class DesTraj:
	def __init__(self, traj_id, traj_name, desTraj, orig_range):
		self.TrajId = traj_id
		self.TrajName = traj_name
		self.DesTraj = self.text2Dict(str(desTraj))
		self.orig_range = json.loads(str(orig_range))
		self.hover_pos = [0.0, 1.0] # [X (horizontal), Y (vertical)]
		self.traj_start = 0
		self.traj_end = 0
		#print "DesTraj __init__: traj_start=%d" %self.traj_start

	def getDesTraj(self, XY=False):
		# Main API


		traj = self.DesTraj
		times = traj.keys()
		times.sort()
		data = []
		for time in times:
			if traj[time]['x'] != None and traj[time]['y'] != None and traj[time]['vx'] != None and traj[time]['vy'] != None:
				data.append(traj[time])
		self.DesTraj = self.process_trajectory(data, False)
		return self.DesTraj


	# Following functions in class DesTraj are all helper functions #
	def process_trajectory(self, data, XY=False):
		time_step = 0.015 # Deliberately making it smaller to let quadrotor fly slower
		v_lim = 0.6
		a_lim = 2

		##### Dimension normalization #####
		x_min = 10000
		x_max = -10000
		y_min = 10000
		y_max = -10000

		from demo_config import phys_range
		import json
		r = json.loads(phys_range)
		#print ("%s" %str(self.orig_range))
		
		x_min = self.orig_range[0][0]
		x_max = self.orig_range[0][1]
		y_min = self.orig_range[1][0]
		y_max = self.orig_range[1][1]
		
		size_x = r[0][1] - r[0][0]
		if XY:
			size_y = r[0][1] - r[0][0]
		else:
			size_y = r[1][1] - r[1][0]
		offset_x = -0.5 * size_x

		if XY:
			offset_y = -0.5 * size_y
		else:
			offset_y = 0.3
			scale_x = size_x / (x_max - x_min)
			scale_y = size_y / (y_max - y_min)

		v_cur = (0, 0)

		traj = []

		for i in range(len(data)):
			x, y = (data[i]["x"], data[i]["y"])
			#print x, y
			x = (x - x_min) * scale_x + offset_x
			y = (y - y_min) * scale_y + offset_y
			traj.append((x, y))
		#print traj
		#traj.append((0, 0))


		# Add take-off head and hovering before drawing the pattern
		# vertical take-off
		head_traj = []
		takeoff_time = 1.5
		takeoff_height = 1.0
		N = int(takeoff_time // time_step)

		for i in range(N):
			if XY:
				head_traj.append((0, takeoff_height * i / N, 0, 0, takeoff_height / takeoff_time, 0))
			else:
				head_traj.append((0, 0, takeoff_height * i / N, 0, 0, takeoff_height / takeoff_time))


		# vertical landing
		landing_traj = []
		landing_time = 1.5
		landing_height = 1.0
		N = int(landing_time // time_step)
		for i in range(N):
			if XY:
				landing_traj.append((0, landing_height * i / N, 0, 0, landing_height / landing_time, 0))
			else:
				landing_traj.append((0, 0, landing_height * i / N, 0, 0, landing_height / landing_time))
		# After taking off, go to initial hover place
		take_off_state = (0.0, 0.0, takeoff_height, 0.0, 0.0, 0.0)
		take_off_hover = 3 #3 seconds
		head_traj.extend([take_off_state] * int(take_off_hover // time_step))
		#print head_traj
		#print "data[0] =", data[0]
		starting_hover = finishing_hover = 3 #3 seconds to signal the starting and ending of trajectory-flying
		if XY:
			starting_hover_traj = [(traj[0][0], traj[0][1], 1.0, 0.0, 0.0, 0.0)] * int(starting_hover // time_step)
			landing_hover_traj = [(traj[-1][0], traj[-1][1], 1.0, 0.0, 0.0, 0.0)] * int(finishing_hover // time_step)
		else:
			starting_hover_traj = [(traj[0][0], 0.0, traj[0][1], 0.0, 0.0, 0.0)] * int(starting_hover // time_step)
			landing_hover_traj = [(traj[-1][0], 0.0, traj[-1][1], 0.0, 0.0, 0.0)] * int(finishing_hover // time_step)
		# move to starting position of the trajectory
		# move to the landing state (same as take-off state)
		moving_time = 3 #3 seconds
		moving_pts = int(moving_time // time_step)
		#print "moving points =", moving_pts
		for t in range(1, moving_pts + 1):
			if XY:
				head_traj.append((t * 1.0 / moving_pts * (traj[0][0] - take_off_state[0]) + take_off_state[0],
				                  t * 1.0 / moving_pts * (traj[0][1] - take_off_state[1]) + take_off_state[1],
				                  1.0,
				                  (traj[0][0] - take_off_state[0]) / moving_pts,
				                  (traj[0][1] - take_off_state[1]) / moving_pts,
				                  0.0))
				landing_traj.append((t * 1.0/ moving_pts * (take_off_state[0] - traj[-1][0]) + traj[-1][0],
				                     t * 1.0/ moving_pts * (take_off_state[1] - traj[-1][1]) + traj[-1][1],
				                     1.0,
				                     (take_off_state[0] - traj[-1][0]) / moving_pts,
				                     (take_off_state[1] - traj[-1][1]) / moving_pts,
				                     0.0))
			else:
				#print "t =", t, "moving_pts =", moving_pts, "traj[0][0] =", traj[0][0], "take_off_state[0] =", take_off_state[0]
				#print t / moving_pts * (traj[0][0] - take_off_state[0]) + take_off_state[0]
				head_traj.append((t * 1.0/ moving_pts * (traj[0][0] - take_off_state[0]) + take_off_state[0],
				                  0.0,
				                  t * 1.0/ moving_pts * (traj[0][1] - take_off_state[2]) + take_off_state[2],
				                  (traj[0][0] - take_off_state[0]) / moving_pts,
				                  0.0,
				                  (traj[0][1] - take_off_state[2]) / moving_pts))
				landing_traj.append((t * 1.0/ moving_pts * (take_off_state[0] - traj[-1][0]) + traj[-1][0],
				                     0.0,
				                     t * 1.0/ moving_pts * (take_off_state[2] - traj[-1][1]) + traj[-1][1],
				                     (take_off_state[0] - traj[-1][0]) / moving_pts,
				                     0.0,
				                     (take_off_state[1] - traj[-1][1]) / moving_pts))

			# add starting hover to head_traj (used to signal the start of trajectory-flying)

		head_traj += starting_hover_traj
		landing_traj = landing_hover_traj + landing_traj



		##### Distance capping #####
		dis_cap = 0.01

		new_traj = []
		i = 0
		#cur_x, cur_y = (0, 0)
		cur_x, cur_y = traj[0]
		while i < len(traj):
			new_traj.append((cur_x, cur_y))
			nxt_x, nxt_y = traj[i]
			if ((cur_x - nxt_x) ** 2 + (cur_y - nxt_y) ** 2) ** 0.5 > dis_cap:
				i -= 1
				ang = math.atan2(nxt_y - cur_y, nxt_x - cur_x)
				cur_x, cur_y = (cur_x + dis_cap * math.cos(ang), cur_y + dis_cap * math.sin(ang))
			i += 1

		sight_f = 20
		length = len(new_traj)
		#print "length:", length

		##### Velocity capping #####
		v_max = []

		for i in range(sight_f):
			v_max.append(0.1)


		for i in range(sight_f, length - sight_f):
			pre_pts = new_traj[(i-sight_f):i]
			post_pts = new_traj[(i+sight_f):(i+sight_f+1)]
			pre_pt = (np.linalg.norm([p[0] for p in pre_pts]), np.linalg.norm([p[1] for p in pre_pts]))
			post_pt = (np.linalg.norm([p[0] for p in post_pts]), np.linalg.norm([p[1] for p in post_pts]))
			cur_pt = new_traj[i]
			pre_pt = (pre_pt[0] - cur_pt[0], pre_pt[1] - cur_pt[1])
			post_pt = (post_pt[0] - cur_pt[0], post_pt[1] - cur_pt[1])
			ang = math.atan2(pre_pt[1] - post_pt[1], pre_pt[0] - post_pt[0])
			v_max.append((v_lim - 0.05) * (math.sin(((math.pi - abs(ang)) / 2.0))) + 0.05)

		for i in range(sight_f):
			v_max.append(0.1)


		for i in range(length - 1, 0, -1):
			dis = ((new_traj[i][0] - new_traj[i-1][0]) ** 2 + (new_traj[i][1] - new_traj[i-1][1]) ** 2) ** 0.5
			v_new = (2 * a_lim * dis + v_max[i] ** 2) ** 0.5
			if v_new < v_max[i-1]:
				v_max[i-1] = v_new

		##### Wait wrt acceleration #####
		#cur_x, cur_y = (0, 0)
		cur_x, cur_y = new_traj[0]
		i = 0
		res_traj = []
		while i < len(new_traj):

			#if i == 0:
				#print cur_x, cur_y
			x, y = new_traj[i]
			while((x-cur_x) ** 2 + (y-cur_y) ** 2) ** 0.5 >= v_max[i] * time_step * 14.0:

				ang = math.atan2((y-cur_y), (x-cur_x))
				cur_vx, cur_vy = v_cur
				target_vx, target_vy = (v_max[i] * math.cos(ang) - cur_vx, v_max[i] * math.sin(ang) - cur_vy)
				ang2 = math.atan2(target_vy, target_vx)
				if (target_vx ** 2 + target_vy ** 2) ** 0.5 > a_lim * time_step:
					v_new = (cur_vx + math.cos(ang2) * a_lim * time_step, cur_vy + math.sin(ang2) * a_lim * time_step)
				else:
					v_new = (v_max[i] * math.cos(ang), v_max[i] * math.sin(ang))
				new_vx, new_vy = v_new
				#print cur_x, cur_y, x, y, v_max[i]
				cur_x += ((cur_vx + new_vx) * time_step / 2.0)
				cur_y += ((cur_vy + new_vy) * time_step / 2.0)
				if XY:
					res_traj.append((cur_x, cur_y, 1.0, new_vx, new_vy, 0))
				else:
					res_traj.append((cur_x, 0, cur_y, new_vx, 0, new_vy))

				v_cur = (new_vx, new_vy)
				#print cur_x, cur_y, cur_vx + new_vx, cur_vx + new_vy
			if i == len(new_traj) - 1:
				break
			nxt_x, nxt_y = new_traj[i+1]
			while (x-cur_x) ** 2 + (y-cur_y) ** 2 > (nxt_x-cur_x) ** 2 + (nxt_y-cur_y) ** 2 and i < len(new_traj) - 2:
				#print "2:", cur_x, cur_y
				x, y = nxt_x, nxt_y
				i += 1
				nxt_x, nxt_y = new_traj[i+1]

			i += 1
		# combine res_traj with the take-off head traj and the landing traj
		res_traj = head_traj + res_traj + landing_traj
		#res_traj = head_traj + landing_traj
		#print "length of head_traj =", len(head_traj)
		#print "length of res_traj =", len(res_traj)
		#print "length of land_traj =", len(landing_traj)
		self.traj_start = len(head_traj)
		self.traj_end = len(res_traj) - len(landing_traj)
		return res_traj



	def text2Dict(self, desTrajText):
		decoder = json.JSONDecoder()
		mTrajList = decoder.decode(desTrajText)
		mTrajDict = {}
		for pt in mTrajList:
			mTrajDict[pt['time']] = {'x': pt['x'], 'y': pt['y'], 'vx': pt['vx'], 'vy': pt['vy']}
		return mTrajDict


	def get_rid_of_nulls(self, traj):
		times = traj.keys()
		times.sort()

		clean_traj = {}
		for t in times:
			if traj[t]['x'] == None or traj[t]['y'] == None or \
			   traj[t]['vx'] == None or traj[t]['vy'] == None:
				#print "null traj[%f] is %s" %(t, str(traj[t]))
				continue

			else:
				clean_traj[t] = traj[t]

		return clean_traj



if __name__ == "__main__":

	# Test: class DesTrajGetter
	dg = DesTrajGetter()
	#d = dg.get_traj_by_name('Traj name')
	d = dg.get_newest_traj()
	traj = d.getDesTraj()
	print "start:", traj[d.traj_start]
	print "end:", traj[d.traj_end]

	x = [elem[0] for elem in traj]
	y = [elem[2] for elem in traj]

	plt.plot(x, y, 'g-')
	plt.title('newest trajectory')
	plt.show()
