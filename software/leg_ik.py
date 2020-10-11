
import numpy as np

class Leg:
	def __init__ (self):
		self.l1 = 0.167
		self.l2 = 0.18
		self.create_range ()
	
	def motor_pos (self, action):
		foot_coord = self.calc_coord(action)
		motor_angle = self.calc_angle(foot_coord)
		return motor_angle
	
	# --- From raw action to cathesian coordinate ---
	def calc_coord (self, action):
		a = np.diag(action)
		u = (np.identity(3)-a) @ self.u_m + a @ self.u_M
		b = (np.identity(3)-a) @ self.b_m + a @ self.b_M
		x = np.linalg.solve (u, b)
		return x
		
	# --- From cathesian coordinate to rotation angle ---
	def calc_angle (self, coord):
		a1 = np.arctan2(coord[1], -coord[2])[0]
		
		d2 = np.sum(np.square(coord))
		d = np.sqrt(d2)
		a3 = np.pi - np.arccos((self.l1*self.l1 + self.l2*self.l2 - d2)/(2*self.l1*self.l2))
		
		a_aux = np.arccos((self.l1*self.l1 - self.l2*self.l2 + d2)/(2*self.l1*d))
		a2 = np.arcsin(coord[0]/d)[0] - a_aux
		return [a1, -a2, -a3]
	
	# --- Creating the work volume ---
	def create_range (self):
		zM = -self.l1*2/3
		ym = 0.1
		l = self.l1+self.l2-0.01
		xm = 0.17
		zm = -np.sqrt(l*l-xm*xm-ym*ym)
		xM = np.sqrt(l*l-ym*ym-zM*zM)
		
		umx, bmx = self.face_from_point([-xm, -ym, zm], [-xm, ym, zm], [-xM, ym, zM])
		uMx, bMx = self.face_from_point([xm, -ym, zm], [xm, ym, zm], [xM, ym, zM])
		
		umy, bmy = self.face_from_point([-xm, -ym, zm], [xm, -ym, zm], [xm, -ym, zM])
		uMy, bMy = self.face_from_point([-xm, ym, zm], [xm, ym, zm], [xm, ym, zM])
		
		umz, bmz = self.face_from_point([-xm, ym, zm], [xm, ym, zm], [xm, -ym, zm])
		uMz, bMz = self.face_from_point([-xm, ym, zM], [xm, ym, zM], [xm, -ym, zM])
		
		self.u_m = np.stack([umx, umy, umz])
		self.u_M = np.stack([uMx, uMy, uMz])
		
		self.b_m = np.asarray([bmx, bmy, bmz]).reshape((3,1))
		self.b_M = np.asarray([bMx, bMy, bMz]).reshape((3,1))
		
	def face_from_point (self, p1, p2, p3):
		p1 = np.asarray(p1).reshape((3,))
		p2 = np.asarray(p2).reshape((3,))
		p3 = np.asarray(p3).reshape((3,))
		v1 = p1-p2
		v2 = p3-p2
		u = np.cross(v1,v2)
		b = np.sum(p2*u)
		return u, b