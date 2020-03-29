# https://lego.github.io/lego-ble-wireless-protocol-docs/
import bleak
import asyncio
import traceback
import struct
import time
import threading
import atexit
import sys



UART_UUID="00001623-1212-efde-1623-785feabcd123"
CHAR_UUID="00001624-1212-efde-1623-785feabcd123"
LEGO_DEVICES={0x0001:"Motor",0x0002:"System Train Motor",0x0005:"Button",0x0008:"Light",0x0014:"Voltage",0x0015:"Current",0x0016:"Piezo Tone (Sound)",0x0017:"RGB Light",0x0022:"External Tilt Sensor",0x0023:"Motion Sensor",0x0025:"Vision Sensor",0x0026:"External Motor with Tacho",0x0027:"Internal Motor with Tacho",0x0028:"Internal Tilt",0x0029:"Duplo Train Motor",0x002A:"Duplo Train Speaker",0x002B:"Duplo Train Color",0x002C:"Duplo Train Speedometer",0x002E:"Technic Control+ Large Motor",0x002F:"Technic Control+ XL Motor",0x0036:"Powered Up Hub IMU Gesture",0x0037:"Remote Button",0x0038:"Remote Signal Level",0x0039:"Powered Up Hub IMU Accelerometer",0x003A:"Powered Up Hub IMU Gyro",0x003B:"Powered Up Hub IMU Position",0x003C:"Powered Up Hub IMU Temperature"}
PORT_INFO_TYPES={"NAME": 0,"VALUE FORMAT":0x80,"RAW Range":0x01,"PCT Range":0x02,"SI Range":0x03,"Symbol":0x04,"MAPPING": 0x05}
PORT_MODE_INFO_MAPPING_BITS=["NA","NA","Discrete","Relative","Absolute","NA","Supports Functional Mapping 2.0","Supports NULL"]
PORT_MODE_INFO_DATASET_TYPES=["8b","16b","32b","float"]
PORT_OUTPUT_FEEDBACK={0x01:"Buffer Empty + Command In Progress",0x02:"Buffer Empty + Command Completed",0x04:"Current Command(s) Discarded",0x08:"Idle",0x10:"Busy/Full"}
RGB_LED_COLORS=["off","magenta","purple","blue","cyan","turquoise","green","yellow","orange","red","white"]
VISION_SENSOR_COLOR_DECODE={0:"black",3:"blue",5:"green",7:"yellow",9:"red",10:"white",255:"UNKNOWN"}
VISION_SENSOR_MODES=["basic_color","proximity","count","reflect","ambience","color_o","rgb_color"]
GYRO_SENSOR_VALUES=["roll","pitch","yaw"]
ACCELEROMETER_SENSOR_VALUES=["x","z","y"]



class Hub:
	_hc={}
	_il={}
	_ol={}
	_hl=[]
	_id=0
	_br=False



	def __init_subclass__(cls):
		Hub._hc[cls._m_id]=cls



	@staticmethod
	def run(f,*a,**kw):
		lp=asyncio.get_event_loop()
		def _end(es=0):
			for h in Hub._hl:
				h.disconnect()
			while (len(Hub._ol.keys())>0):
				pass
			Hub._br=True
			while (lp.is_running()==True):
				pass
			lp.stop()
			lp.close()
			sys.exit(es)
		def _run(f,a,kw):
			try:
				f(*a,**kw)
				_end(es=0)
			except Exception as e:
				traceback.print_exception(None,e,e.__traceback__)
				_end(es=1)
		thr=threading.Thread(target=_run,args=(f,a,kw),kwargs={})
		thr.start()
		async def _loop():
			while (Hub._br==False):
				while (len(Hub._ol.keys())==0):
					if (Hub._br==True):
						return
					await asyncio.sleep(0.001)
				id_=list(Hub._ol.keys())[0]
				t,a,kw=Hub._ol.pop(id_);
				if (t==0):
					kw["loop"]=asyncio.get_event_loop()
					Hub._il[id_]=await bleak.discover(*a,**kw)
				elif (t==1):
					kw["loop"]=asyncio.get_event_loop()
					dv=bleak.BleakClient(*a,**kw)
					await dv.connect()
					Hub._il[id_]=dv
				elif (t==2):
					await a[0].start_notify(CHAR_UUID,*a[1:],**kw)
				elif (t==3):
					await a[0].write_gatt_char(CHAR_UUID,*a[1:],**kw)
				else:
					print(f"UNKNOWN: {id_} => [{t},{a},{kw}]")
		atexit.register(_end)
		lp.run_until_complete(_loop())



	@staticmethod
	def find(cfl=[],i=5,t=None):
		hl=[]
		al=[]
		while (i>0 and len(hl)==0):
			dl=Hub._wait([0,[],{"timeout":1}])
			dl=[d for d in dl if UART_UUID in d.metadata["uuids"]]
			for d in dl:
				dt=list(d.metadata["manufacturer_data"].values())[0]
				if (d.address not in al):
					if (t==None):
						for m_id in Hub._hc.keys():
							if (dt[1]==m_id):
								print(d.address)
								dc=Hub._wait([1,[],{"address":d.address}])
								h=Hub._hc[m_id](dc)
								h._init()
								hl+=[h]
								al+=[d.address]
								break
					else:
						if (dt[1]==t._m_id):
							print(d.address)
							dc=Hub._wait([1,[],{"address":d.address}])
							h=t(dc)
							h._init()
							hl+=[h]
							al+=[d.address]
							break
			i-=1
		if (len(cfl)==0):
			return hl
		o=[]
		for cf in cfl:
			o+=[[]]
			for h in hl:
				pl=h.get_port(list(cf.keys()))
				ok=True
				for i in range(0,len(pl)):
					if (pl[i]._id!=list(cf.values())[i]._id):
						ok=False
						break
				if (ok==True):
					o[-1]+=[h]
		return o



	@staticmethod
	def _wait(dt,w=True,r=True):
		id_=Hub._id+0
		Hub._ol[id_]=dt
		Hub._id+=1
		if (w==False):
			return None
		while (id_ in Hub._ol.keys()):
			pass
		if (r==False):
			return None
		while (id_ not in Hub._il.keys()):
			pass
		return Hub._il.pop(id_)



	def __init__(self,d,nm):
		self.d=d
		self._ol=[]
		self._port_dt={}
		self._dl={}
		self.name=nm
		Hub._hl+=[self]



	def connections(self):
		for p in self._port_dt.keys():
			print(f"{p} => {self._port_dt[p]['name']}")
		return None



	def get_port(self,pl,wait=True):
		all_=False
		if (type(pl)==int):
			pl=[pl]
		if (type(pl)==str):
			if (pl=="all"):
				all_=True
				pl=[*self._port_dt.keys()]
			else:
				pl=pl.split(",")
		for p in pl:
			while (wait==True and (p not in self._port_dt.keys() or "_ready" not in self._port_dt[p].keys() or self._port_dt[p]["_ready"]==False)):
				pass
			if (all_==True and len(pl)!=len(self._port_dt.keys())):
				return self.get_port("all",wait=wait)
		return [self._port_dt[p]["driver"] for p in pl]



	def get_port_modes(self,pl,wait=True):
		all_=False
		if (type(pl)==int):
			pl=[pl]
		if (type(pl)==str):
			if (pl=="all"):
				all_=True
				pl=[*self._port_dt.keys()]
			else:
				pl=pl.split(",")
		for p in pl:
			while (wait==True and (p not in self._port_dt.keys() or "_modes_ready" not in self._port_dt[p].keys() or self._port_dt[p]["_modes_ready"]==False)):
				pass
			if (all_==True and len(pl)!=len(self._port_dt.keys())):
				return self.get_port("all",wait=wait)
		return [self._port_dt[p]["modes"] for p in pl]



	def wait_until_data(self,pl):
		all_=False
		if (type(pl)==int):
			pl=[pl]
		if (type(pl)==str):
			if (pl=="all"):
				all_=True
				pl=[*self._port_dt.keys()]
			else:
				pl=pl.split(",")
		self.get_port(pl,wait=True)
		for p in pl:
			while ((self._port_dt[p]["driver"]._dt==False)):
				pass
			if (all_==True and len(pl)!=len(self._port_dt.keys())):
				self.get_port("all",wait=wait)
				break



	def disconnect(self):
		self._send([0x00,0x02,0x01],wait=False)



	def _init(self):
		def _msg(s,dt):
			thr=threading.Thread(target=self._msg,args=(s,dt),kwargs={})
			thr.start()
		Hub._wait([2,[self.d,_msg],{}],r=False)



	def _send(self,dt,wait=True):
		Hub._wait([3,[self.d,bytearray([len(dt)+1]+dt)],{}],w=wait,r=False)



	def _msg(self,s,dt):
		try:
			dt=list(dt)[2:]
			t,dt=dt[0],dt[1:]
			if (t==0x02):
				pass
			elif (t==0x04):
				p,e,dt=dt[0],dt[1],dt[2:]
				if (e!=0):
					id_,dt=dt[0],dt[2:]
					if (p not in self._port_dt.keys()):
						self._port_dt[p]={}
					self._port_dt[p]["_ready"]=False
					self._port_dt[p]["id"]=id_
					self._port_dt[p]["name"]=(LEGO_DEVICES[id_] if id_ in LEGO_DEVICES.keys() else f"UNKNOWN #{p}")
					if (e==2):
						p0,p1=dt
						self._port_dt[p]["virtual"]=(p0,p1)
						print(p,p0,p1)
					self._port_dt[p]["driver"]=HubDriver.get(id_)(self,p)
					self._dl[p]=self._port_dt[p]["driver"]
					self._send([0x00,0x21,p,0x01],wait=False)
					self._port_dt[p]["_ready"]=True
			elif (t==0x05):
				ct,ec=dt
				if (ec==1):
					print(f"ERROR: {hex(ct)} => ACK")
				elif (ec==2):
					print(f"ERROR: {hex(ct)} => MACK")
				elif (ec==3):
					print(f"ERROR: {hex(ct)} => Buffer Overflow")
				elif (ec==4):
					print(f"ERROR: {hex(ct)} => Timeout")
				elif (ec==5):
					print(f"ERROR: {hex(ct)} => Command Not Recognised")
				elif (ec==6):
					print(f"ERROR: {hex(ct)} => Invalid Use")
				elif (ec==7):
					print(f"ERROR: {hex(ct)} => Overcurrent")
				elif (ec==8):
					print(f"ERROR: {hex(ct)} => Internal Error")
			elif (t==0x43):
				p,m,dt=dt[0],dt[1],dt[2:]
				if ("modes" not in self._port_dt[p].keys()):
					self._port_dt[p]["modes"]={}
					self._port_dt[p]["_modes_ql"]=0
					self._port_dt[p]["_modes_ready"]=False
				if (m==1):
					cl,im,om,dt=dt[0],dt[2]+dt[3]*256,dt[4]+dt[5]*256,dt[6:]
					i=0
					for a in ["output","input","combinable","synchronizable"]:
						self._port_dt[p][a]=cl&1<<i
						i+=1
					for i in range(0,16):
						if (im&(1<<i)):
							if (i not in self._port_dt[p]["modes"].keys()):
								self._port_dt[p]["modes"][i]={}
							self._port_dt[p]["modes"][i]["input"]=True
						if (om&(1<<i)):
							if (i not in self._port_dt[p]["modes"].keys()):
								self._port_dt[p]["modes"][i]={}
							self._port_dt[p]["modes"][i]["output"]=True
					if (self._port_dt[p]["combinable"]==1):
						self._send([0x00,0x21,p,0x02],wait=False)
						self._port_dt[p]["_modes_ql"]+=1
					for m in self._port_dt[p]["modes"].keys():
						for v in PORT_INFO_TYPES.values():
							self._send([0x00,0x22,p,m,v],wait=False)
							self._port_dt[p]["_modes_ql"]+=1
					if (self._port_dt[p]["_modes_ql"]==0):
						self._port_dt[p]["_modes_ready"]=True
				else:
					self._port_dt[p]["mode_combinations"]=[]
					if (len(dt)>0):
						mc,dt=dt[0]+dt[1]*256,dt[2:]
						while (mc!=0):
							cml=[]
							for i in range(16):
								if (mc&(1<<i)):
									cml+=[i]
							self._port_dt[p]["mode_combinations"]+=[cml]
							if (len(dt)==0):
								break
							else:
								mc,dt=dt[0]+dt[1]*256,dt[2:]
					self._port_dt[p]["_modes_ql"]-=1
					if (self._port_dt[p]["_modes_ql"]==0):
						self._port_dt[p]["_modes_ready"]=True
			elif (t==0x44):
				p,m,mt,dt=dt[0],dt[1],dt[2],dt[3:]
				if (mt==0):
					self._port_dt[p]["modes"][m]["name"]="".join([chr(b) for b in dt if b!=0])
				elif (mt==1):
					self._port_dt[p]["modes"][m]["raw_range"]={"min":struct.unpack("<f",bytearray(dt[0:4]))[0],"max":struct.unpack("<f",bytearray(dt[4:]))[0]}
				elif (mt==2):
					self._port_dt[p]["modes"][m]["pct_range"]={"min":struct.unpack("<f",bytearray(dt[0:4]))[0],"max":struct.unpack("<f",bytearray(dt[4:]))[0]}
				elif (mt==3):
					self._port_dt[p]["modes"][m]["si_range"]={"min":struct.unpack("<f",bytearray(dt[0:4]))[0],"max":struct.unpack("<f",bytearray(dt[4:]))[0]}
				elif (mt==4):
					self._port_dt[p]["modes"][m]["symbol"]="".join([chr(b) for b in dt if b!=0])
				elif (mt==5):
					self._port_dt[p]["modes"][m]["input_mapping"]=[PORT_MODE_INFO_MAPPING_BITS[i] for i in range(8) if (dt[0]>>i)&1]
					self._port_dt[p]["modes"][m]["output_mapping"]=[PORT_MODE_INFO_MAPPING_BITS[i] for i in range(8) if (dt[1]>>i)&1]
				elif (mt==128):
					self._port_dt[p]["modes"][m]["datasets"]=dt[0]
					self._port_dt[p]["modes"][m]["dataset_type"]=PORT_MODE_INFO_DATASET_TYPES[dt[1]]
					self._port_dt[p]["modes"][m]["dataset_total_figures"]=dt[2]
					self._port_dt[p]["modes"][m]["dataset_decimals"]=dt[3]
				else:
					print(mt)
				self._port_dt[p]["_modes_ql"]-=1
				if (self._port_dt[p]["_modes_ql"]==0):
					self._port_dt[p]["_modes_ready"]=True
			elif (t==0x45 or t==0x46):
				p,dt=dt[0],dt[1:]
				self._port_dt[p]["driver"]._parse_caps(dt)
			elif (t==0x47 or t==0x48):
				pass
			elif (t==0x82):
				for i in range(0,len(dt),2):
					l=[]
					j=0
					for s in PORT_OUTPUT_FEEDBACK.values():
						if (dt[i+1]&1<<j>0):
							l+=[s]
						j+=1
					print(f"FEEDBACK: port#{dt[i]} => {l}")
			else:
				print(hex(t))
		except Exception as e:
			traceback.print_exception(None,e,e.__traceback__)



class HubDriver:
	_dl={}



	@staticmethod
	def get(id_):
		if (id_ in HubDriver._dl.keys()):
			return HubDriver._dl[id_]
		return HubDriver



	def __init__(self,h,p,_id=-1):
		self.h=h
		self.p=p
		self._id=_id
		self._cl=[]
		self._dt=False



	def __init_subclass__(cls):
		HubDriver._dl[cls._id]=cls



	def name(self):
		return LEGO_DEVICES[self.h._port_dt[self.p]["id"]]



	def _setup_caps(self,cl):
		self._cl=cl
		self.value=None
		if (len(cl)==0):
			pass
		elif (len(cl)==1):
			self.value={}
			self.h._send([0x00,0x41,self.p,cl[0][0],cl[0][2],0,0,0,1])
		else:
			self.value={}
			self.h._send([0x00,0x42,self.p,0x02])
			for i in range(0,len(cl)):
				self.h._send([0x00,0x41,self.p,cl[i][0],cl[i][2],0,0,0,1])
			b=[]
			for i in range(0,len(cl)):
				for j in range(cl[i][3]):
					b+=[16*cl[i][0]+j]
			self.h._send([0x00,0x42,self.p,0x01,0]+b)
			self.h._send([0x00,0x42,self.p,0x03])



	def _parse_caps(self,dt):
		if (len(self._cl)==0):
			self.value=dt
		elif (len(self._cl)==1):
			for i in range(0,self._cl[0][3]):
				if (self._cl[0][3]==1):
					self.value[self._cl[0][1]]=self._to_int(dt[i*self._cl[0][4]:(i+1)*self._cl[0][4]],self._cl[0][4])
				else:
					if (self._cl[0][1] not in self.value):
						self.value[self._cl[0][1]]={}
					self.value[self._cl[0][1]][i]=self._to_int(dt[i*self._cl[0][4]:(i+1)*self._cl[0][4]],self._cl[0][4])
			dt=[]
		else:
			m,dt=dt[1],dt[2:]
			i=0
			for j in range(0,len(self._cl)):
				for k in range(0,self._cl[j][3]):
					if (m&(1<<i)):
						if (self._cl[j][3]==1):
							self.value[self._cl[j][1]],dt=self._to_int(dt[0:self._cl[j][4]],self._cl[j][4]),dt[self._cl[j][4]:]
						else:
							if (self._cl[j][1] not in self.value):
								self.value[self._cl[j][1]]={}
							self.value[self._cl[j][1]][k],dt=self._to_int(dt[0:self._cl[j][4]]),dt[self._cl[j][4]:]
					i+=1
			if (len(dt)>0):
				print("EXTRA: "+str(dt))
		self._parse_value()
		self._dt=True



	def _to_int(self,dt,l):
		if (l==1):
			return dt[0]
		if (l==2):
			return struct.unpack("<H",bytearray(dt))[0]
		if (l==4):
			return struct.unpack("<I",bytearray(dt))[0]
		return None



class PoweredUpHub(Hub):
	_m_id=0x41
	_name="HUB NO.4"



	@staticmethod
	def find(*a,**kw):
		kw["t"]=PoweredUpHub
		return Hub.find(*a,**kw)



	def __init__(self,d):
		super().__init__(d,self.__class__._name)



class PoweredUpRemote(Hub):
	_m_id=0x42
	_name="Handset"



	@staticmethod
	def find(*a,**kw):
		kw["t"]=PoweredUpRemote
		return Hub.find(*a,**kw)



	def __init__(self,d):
		super().__init__(d,self.__class__._name)



class BoostHub(Hub):
	_m_id=0x40
	_name="LEGO Move Hub"



	@staticmethod
	def find(*a,**kw):
		kw["t"]=BoostHub
		return Hub.find(*a,**kw)



	def __init__(self,d):
		super().__init__(d,self.__class__._name)



class DuploTrainHub(Hub):
	_m_id=0x20
	_name="Train Base"



	@staticmethod
	def find(*a,**kw):
		kw["t"]=DuploTrainHub
		return Hub.find(*a,**kw)



	def __init__(self,d):
		super().__init__(d,self.__class__._name)



class CPlusHub(Hub):
	_m_id=0x80
	_name="Control+ Hub"



	@staticmethod
	def find(*a,**kw):
		kw["t"]=CPlusHub
		return Hub.find(*a,**kw)



	def __init__(self,d):
		super().__init__(d,self.__class__._name)



class LargeMotor(HubDriver):
	_id=0x2e



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(1,"speed",1,1,1),(2,"pos",1,1,4)])
		self._w_r=False



	def _parse_value(self):
		if (self._w_r==True):
			if (self.value["speed"]<=2):
				self._w_r=False
				self.h._send([0x00,0x81,self.p,0x11,0x51,0,0])



	def _to_bytes(self,sp):
		return (100 if sp>100 else (sp&255 if sp<0 else sp))



	def set_pos(self,ps,sp,m_pw=50,e_st=1):
		self.h._send([0x00,0x81,self.p,0x11,0x0d]+list(struct.pack("i",ps))+[self._to_bytes(sp),m_pw,(0 if e_st==0 else (126 if e_st==1 else 127)),3])



	def rotate(self,d,sp,m_pw=50,e_st=1):
		self.h._send([0x00,0x81,self.p,0x11,0x0b]+list(struct.pack("i",d))+[self._to_bytes(sp),m_pw,(0 if e_st==0 else (126 if e_st==1 else 127)),3])



	def set_speed(self,sp):
		self.h._send([0x00,0x81,self.p,0x11,0x51,0,self._to_bytes(sp)])



	def wait_until_resistance(self):
		self._w_r=True
		while (self._w_r==True):
			pass



class XLMotor(HubDriver):
	_id=0x2f



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(1,"speed",1,1,1),(2,"pos",1,1,4)])
		self._w_r=False



	def _parse_value(self):
		if (self._w_r==True):
			if (self.value["speed"]<=2):
				self._w_r=False
				self.h._send([0x00,0x81,self.p,0x11,0x51,0,0])



	def _to_bytes(self,sp):
		return (100 if sp>100 else (sp&255 if sp<0 else sp))



	def set_pos(self,ps,sp,m_pw=50,e_st=1):
		self.h._send([0x00,0x81,self.p,0x11,0x0d]+list(struct.pack("i",ps))+[self._to_bytes(sp),m_pw,(0 if e_st==0 else (126 if e_st==1 else 127)),3])



	def rotate(self,d,sp,m_pw=50,e_st=1):
		self.h._send([0x00,0x81,self.p,0x11,0x0b]+list(struct.pack("i",d))+[self._to_bytes(sp),m_pw,(0 if e_st==0 else (126 if e_st==1 else 127)),3])



	def set_speed(self,sp):
		self.h._send([0x00,0x81,self.p,0x11,0x51,0,self._to_bytes(sp)])



	def wait_until_resistance(self):
		self._w_r=True
		while (self._w_r==True):
			pass



class ExternalTachoMotor(HubDriver):
	_id=0x26



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(1,"speed",1,1,1),(2,"pos",1,1,4)])
		self._w_r=False



	def _parse_value(self):
		if (self._w_r==True):
			if (self.value["speed"]<=2):
				self._w_r=False
				self.h._send([0x00,0x81,self.p,0x11,0x51,0,0])



	def _to_bytes(self,sp):
		return (100 if sp>100 else (sp&255 if sp<0 else sp))



	def set_pos(self,ps,sp,m_pw=50,e_st=1):
		self.h._send([0x00,0x81,self.p,0x11,0x0d]+list(struct.pack("i",ps))+[self._to_bytes(sp),m_pw,(0 if e_st==0 else (126 if e_st==1 else 127)),3])



	def rotate(self,d,sp,m_pw=50,e_st=1):
		self.h._send([0x00,0x81,self.p,0x11,0x0b]+list(struct.pack("i",d))+[self._to_bytes(sp),m_pw,(0 if e_st==0 else (126 if e_st==1 else 127)),3])



	def set_speed(self,sp):
		self.h._send([0x00,0x81,self.p,0x11,0x51,0,self._to_bytes(sp)])



	def wait_until_resistance(self):
		self._w_r=True
		while (self._w_r==True):
			pass



class VisionSensor(HubDriver):
	_id=0x25



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._md=0
		self._setup_caps([(0,"basic_color",1,1,1)])



	def _parse_value(self):
		if ("basic_color" in self.value):
			self.value["basic_color"]=VISION_SENSOR_COLOR_DECODE[self.value["basic_color"]]
		if ("rgb_color" in self.value):
			self.value["rgb_color"]={"red":self.value["rgb_color"][0],"green":self.value["rgb_color"][1],"blue":self.value["rgb_color"][2]}
		pass



	def set_mode(self,m):
		if (type(m)==str):
			m=VISION_SENSOR_MODES.index(m)
		if (m!=self._md):
			self._md=m
			self._setup_caps([(m,VISION_SENSOR_MODES[m],1,(3 if m==6 else 1),(4 if m==2 else (2 if m in [6,7] else 1)))])



class RGBLed(HubDriver):
	_id=0x17



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._md=0
		self._setup_caps([(0,"preset_color",0,0,0)])



	def _parse_value(self):
		pass



	def set_color(self,c):
		if (type(c)==str):
			c=RGB_LED_COLORS.index(c.lower())
		if (type(c)==int):
			if (self._md==1):
				self._setup_caps([(0,"preset_color",0,0,0)])
				self.h._send([0x00,0x41,self.p,0x00,0,0,0,0,0])
				self._md=0
			self.h._send([0x00,0x81,self.p,0x11,0x51,0x00,c])
		else:
			if (self._md==0):
				self._setup_caps([(1,"rgb_color",0,0,0)])
				self._md=1
			self.h._send([0x00,0x81,self.p,0x11,0x51,0x01]+c)



class GyroSensor(HubDriver):
	_id=0x3a



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(0,"gyro",1,3,2)])
		self._ld_tm=-1
		self._l_dt=[None]*3
		self._off=[0]*3
		self._off_l=[None]*3
		self._g_dt={GYRO_SENSOR_VALUES[i]:0 for i in range(0,3)}



	def _parse_value(self):
		tm=time.time()
		if (self._ld_tm==-1):
			self._ld_tm=tm+0
		df=tm-self._ld_tm
		self._ld_tm=tm+0
		for i in range(3):
			v=(self.value["gyro"][i]+2**15)%2**16-2**15
			if (self._l_dt[i]!=None and abs(v-self._l_dt[i])>=25):
				self._off[i]+=int((v-self._l_dt[i])/655.36*100)/100
				self._l_dt[i]=v
			if (self._l_dt[i]==None):
				self._l_dt[i]=v
			self._g_dt[GYRO_SENSOR_VALUES[i]]+=self._off[i]+0
		self.value={"gyro":self._g_dt}



class TemperatureSensor(HubDriver):
	_id=0x3c



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(0,"temperature",1,1,2)])



	def _parse_value(self):
		self.value["temperature"]=self.value["temperature"]/10
		pass



class BatteryCurrent(HubDriver):
	_id=0x15



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(0,"current",1,1,2)])



	def _parse_value(self):
		pass



class BatteryVoltage(HubDriver):
	_id=0x14



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(0,"voltage",1,1,2)])



	def _parse_value(self):
		pass



class AccelerometerSensor(HubDriver):
	_id=0x39



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(0,"acceleration",1,3,2)])



	def _parse_value(self):
		dt={}
		for i in range(3):
			dt[ACCELEROMETER_SENSOR_VALUES[i]]=(self.value["acceleration"][i]/655.36+50)%100-50
		self.value={"acceleration":dt}
		pass



class PositionSensor(HubDriver):
	_id=0x3b



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(0,"position",1,3,2)])



	def _parse_value(self):
		dt={}
		for i in range(3):
			dt[i]=(self.value["position"][i]/255+128)%255-128
		self.value={"position":dt}
		pass



class GestureSensor(HubDriver):
	_id=0x36



	def __init__(self,h,p):
		super().__init__(h,p,self.__class__._id)
		self._setup_caps([(0,"gesture",1,1,1)])



	def _parse_value(self):
		pass
