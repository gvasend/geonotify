
"""
geosim.py

This module provides a simulation context to allow the geonotification service to be exercised and tested. The simulation combined randomly generated uav's in a given area as well as integrates an ADSB feed via a commercial API. Two scenario files are available: test_scenario.json which defines the scenario and uav_data.json which defines performance data for various uav models.

Future considerations:
    - This code is a rough prototype. I went for breadth over depth. Needs to be fleshed out in more detail.
    - Needs V&V/more testing. 
    - Treatment of altitude needs work. AGL vs. geometric altitude is not fully implemented.
    - Need a more complete representation of time. Currently time is in seconds relative to the start of the simulation.
    - Support repeatability. This could be handled with either a data file of uav flights or a random number seed. Perhaps both.
    - Migrate service to more closely align to the ultimate target architecture which would likely include moving the service to its own process.
    - Investigate scalability. The number of UAVs in flight may grow significantly. The service could allow compute elasticity by subdividing 
      area coverage among processors. 
    - FAA/EASA seem to be moving towards a integrated/networked airspace which could evolve into a federation of such services exchanging data.
    - Consider what fidelity of dead reckoning is required by the service (e.g. wind). 
    - Consider other notification zone shapes. Currently it is represented as a sphere.
    - Drone operations could vary significantly from a focused objective to eratic operation. Machine Learning might be used to predict such behavior.
"""

from pubsub import pub
import simpy, json, time, random, math, geopy.distance, sys
from datetime import datetime
import geonotify

scenario = None
sim_speed = 1.0
ADSB_POLL_INTERVAL = 120.0 # rate at which ADSB is polled 
UAV_UPDATE_INTERVAL = 30.0 # rate at which simulated uav tracks are updated
SIM_PACE_DELAY = 0.1    # SIM_PACE_DELAY allows the simulation to idle while synchronizing between simulated and real-time.
                        # Need to experiment with different values to see what provides best performance.

# Look for the scenario file on the command line
if len(sys.argv) > 1:
    scenario_file = sys.argv[1]
else:
    scenario_file = 'test_scenario.json'
    
print('Scenario file:',scenario_file)


class Aircraft():
    endurance = 30.0
    ac_class = 'unknown'
    latitude = 0.0
    longitude = 0.0
    launch_lat = 0.0
    launch_lon = 0.0
    speed = 0.0
    heading = 0.0
    altitude = 0.0
    ID = 'unknown',
    cruise_speed = 0.0
    def __init__(self, **kwargs):
        self.state = 'off'
        self.latitude=kwargs.get('latitude',0.0)
        self.longitude=kwargs.get('longitude',0.0)
        self.endurance=kwargs.get('endurance',30.0)
        self.agl_altitude = kwargs.get('agl_altitude',0.0)
        self.control_lat = self.latitude    # mark control location as initial launch point
        self.control_lon = self.longitude
        self.control_alt = self.agl_altitude
        self.speed = kwargs.get('speed',10.0)
        self.heading = kwargs.get('heading',10.0)
        self.climb_speed = kwargs.get('climb_speed',10.0)
        self.tlu = env.now
        self.start_time = env.now
    def run(self):
        while True:
            self.update_adsb()
#            print(self.ID,'endurance',self.endurance)
            yield env.timeout(UAV_UPDATE_INTERVAL)
            self.endurance -= (env.now-self.tlu)/60.0   # endurance is in minutes
            self.tlu = env.now
            if self.endurance <= 0.0:
                self.agl_altitude = 0
                self.speed = 0
                self.climb_speed = 0
                self.update_adsb()
                print('uav %s landed start %f end %f' % (self.ID, self.start_time, env.now))
                return
            else:
                self.latitude, self.longitude, self.agl_altitude = next_position(1.0, self.heading, self.speed, self.latitude, self.longitude, self.agl_altitude, self.climb_speed)
    
    # broadcast uav position
    def update_adsb(self):
        pub.sendMessage('adsb', ID=self.ID, ac_class=self.ac_class, latitude=self.latitude, longitude=self.longitude, agl_altitude=self.agl_altitude, velocity=(self.heading,self.speed,self.climb_speed), control_lat=self.control_lat, control_lon=self.control_lon, control_alt=self.control_alt)        

# Dead reckon the next position 
def next_position(delta_t, brng, speed, current_lat, current_lon, current_altitude, climb_speed):
    R = 6378.1 #Radius of the Earth
    brng = math.radians(brng)
    d = (speed/3600.0)*delta_t
#    print('distance traveled (meters)',d*1000)
    lat1 = math.radians(current_lat) #Current lat point converted to radians
    lon1 = math.radians(current_lon) #Current long point converted to radians

    lat2 = math.asin( math.sin(lat1)*math.cos(d/R) +
         math.cos(lat1)*math.sin(d/R)*math.cos(brng))

    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1),
                 math.cos(d/R)-math.sin(lat1)*math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    alt = current_altitude + climb_speed*delta_t
    return (lat2,lon2,alt)
    
class Uav(Aircraft):
    model = 'unknown'
    def __init__(self,uav_model='unkown',ID=None, **kwargs):
        super(Uav, self).__init__(**kwargs)
        self.performance = uav_dct[uav_model] 
        self.ID = ID
        self.model = uav_model
        self.ac_class = 'uav'

class MannedAircraft(Aircraft):
    def __init__(self, **kwargs):
        super(Aircraft, self).__init__(kwargs)
        self.ac_class = 'manned ac'
        return

def UavGenerator(scenario):
    # generates random uavs
    for n in range(scenario['start_number_uav']):
        yield env.timeout(scenario_dct['average_uav_arrival_time'])
        model = random.choice(list(uav_dct.keys()))
        # Compute random speed, heading, target altitude, starting location
        speed = random.uniform(10.0, uav_dct[model]['max_speed'])
        endurance = random.uniform(10.0, uav_dct[model]['endurance'])
        heading = random.uniform(0.0, 360.0)
        latitude = random.normalvariate((scenario['lower_left_lat']+scenario['upper_right_lat'])/2.0, scenario['loc_std_dev'])
        longitude = random.normalvariate((scenario['lower_left_lon']+scenario['upper_right_lon'])/2.0, scenario['loc_std_dev'])
        agl = random.uniform(10.0,1000.0) # will generate some flights that are out of bounds
        uav = Uav(uav_model=model,ID='%s-%d'%(model,n),speed=speed,heading=heading,endurance=endurance,latitude=latitude,longitude=longitude, agl_altitude=agl)
        print('generating uav',uav.ID)
        env.process(uav.run())

show_interval = 60.0   # How often to print current time
last_time = 0.0
def pace_simulation():
    # Pace simulation to wall clock time based on the simulation speed. e.g. 1.0 will match sim time to real-time. 2.0 will run sim time and twice wall clock time.
    global last_time
    while True:
        simnow = env.now
        now = (time.time()-start_time)
        if simnow-last_time > show_interval:
            print('clock time:',now,'sim time:',simnow)
            last_time = simnow
        if simnow > now*sim_speed:
            pass
        else:
            yield env.timeout(((now*sim_speed)-simnow)/sim_speed)
            time.sleep(SIM_PACE_DELAY)

def load_uav_data(filename):
    with open(filename,'r') as inf:
        uav_data_str = inf.read()
    uav_data = json.loads(uav_data_str)
    print('UAV data',uav_data)
    return uav_data
    
def execute_scenario(filename):
    global env, start_time, scenario_dct, uav_dct
    with open(filename,'r') as inf:
        scen_str = inf.read()
    print('scenario string:',scen_str)
    scenario_dct = json.loads(scen_str)
    uav_dct = load_uav_data(scenario_dct['uav_data_file'])
    print('scenario:',scenario_dct)
    for nz in scenario_dct['notification_zones']:
        pub.sendMessage('create_zone', ID=nz['ID'], start_time=nz['start_time'], end_time=nz['end_time'], center=(nz['center_lat'],nz['center_lon']), radius=nz['radius'], notify=nz['notify_endpoint'])
    adsb_lat = scenario_dct['adsb_lat']
    adsb_lon = scenario_dct['adsb_lon']
    adsb_radius = scenario_dct['adsb_radius']
    env = simpy.Environment()
    geonotify.env = env
    env.process(get_adsb(adsb_lat,adsb_lon,adsb_radius))
    env.process(geonotify.check_intrusions())
    env.process(UavGenerator(scenario_dct))
    env.process(pace_simulation())
    start_time = time.time()
    env.run(until=scenario_dct['scenario_duration'])
    
import requests
"""
Sample API results:
{"hex":"407b70","type":"adsb_icao","flight":"WUK3722 ","r":"G-WUKO","t":"A21N","alt_baro":8000,"alt_geom":7675,"gs":267.4,"ias":252,"tas":280,"mach":0.440,"wd":44,"ws":13,"oat":-6,"tat":4,"track":29.82,"track_rate":0.00,"roll":-0.35,"mag_heading":30.06,"true_heading":30.49,"baro_rate":0,"geom_rate":0,"squawk":"7454","emergency":"none","category":"A3","nav_qnh":1012.8,"nav_altitude_mcp":8000,"nav_heading":49.92,"lat":50.940170,"lon":-0.727325,"nic":8,"rc":186,"seen_pos":0.087,"version":2,"nic_baro":1,"nac_p":9,"nac_v":4,"sil":3,"sil_type":"perhour","gva":2,"sda":3,"alert":0,"spi":0,"mlat":[],"tisb":[],"messages":94148623,"seen":0.0,"rssi":-9.8,"dst":33.352,"dir":198.0}
"""

# Pull data from ADSB to add additional live tracks.
def get_adsb(adsb_lat,adsb_lon,adsb_radius):

    url = "https://adsbexchange-com1.p.rapidapi.com/v2/lat/%f/lon/%f/dist/%d/" % (adsb_lat,adsb_lon,adsb_radius)
    print('url',url)

    headers = {
	    "X-RapidAPI-Key": scenario_dct['adsb_key'],
    	"X-RapidAPI-Host": "adsbexchange-com1.p.rapidapi.com"
    }
    while True:
        response = requests.request("GET", url, headers=headers)
        for track in response.json()['ac']:
            alt = track.get('alt_geom',0.0)
            true_heading = track.get('true_heading',0.0)
            ID = track.get('flight',track['hex'])
            gs = track.get('gs',0.0)
    #        print('adsb track', track)
            pub.sendMessage('adsb', ID=ID, ac_class='manned ac', latitude=track['lat'], longitude=track['lon'], agl_altitude=alt, velocity=(true_heading,gs,track['rc']))
        yield env.timeout(ADSB_POLL_INTERVAL)
        
# Simulated alert receivers

def receive_alert_svc(message=''):
    print('General alert received>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>',message,'<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')
    
def gv_receive_alert_svc(message=''):
    print('gvasend alert received>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>',message,'<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')
    
def js_receive_alert_svc(message=''):
    print('jsmith alert received>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>',message,'<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')
    
pub.subscribe(receive_alert_svc, 'alert')                   # Generate an alert message to the user
pub.subscribe(gv_receive_alert_svc, 'alert.gvasend')                   # Generate an alert message to the user
pub.subscribe(js_receive_alert_svc, 'alert.jsmith')                   # Generate an alert message to the user



execute_scenario(scenario_file)

    
