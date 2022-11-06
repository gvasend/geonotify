
from pubsub import pub
import random, math, geopy.distance

NOTIFICATION_CHECK_INTERVAL = 30.0
TRACK_TIMEOUT = 180.0

#    print(response.text)
    
"""
Simulation of the geonotification service. The service is encapsulated using pypubsub which implements a publish subscribe mechanism local to the
process.

Track parameters are based on the current set of Remote ID parameters

A unique identifier for the drone. Operators of a Standard Remote ID Drone may choose whether to use the drone's serial number or a session ID (an alternative form of identification discussed below that provides additional privacy to the operator) as the unique identifier;
An indication of the drone's latitude, longitude, geometric altitude, and velocity;
An indication of the control station's latitude, longitude, and geometric altitude;
A time mark; and
An emergency status indication.

Note: AGL is temporarily used instead of geometric altitude
"""
env = None  # This will be set to the simulation environment
notify_lst = ['uav','manned ac']  # Specify what types of tracks will be reported
tracks = {}
notification_zones = {}
def create_notification_zone(ID, start_time=0, end_time=3600.0, center=(0,0), radius=0):
    try:
        notification_zones[ID] = {'ID':ID, 'start_time': start_time, 'end_time': end_time, 'center': center, 'radius':radius, 'status':'created','inside':[]}
        pub.sendMessage('alert',message='Created notification zone: %s' %ID)
    except:
        pub.sendMessage('alert',message='exception occurred attempting to create notification zone: %s' %ID)
        
def delete_notification_zone(ID):
    try:
        del notification_zones[ID]
        pub.sendMessage('alert','Deleted notification zone: %s' %ID)
    except:
        pub.sendMessage('alert',message='attempt to delete a non-existant notification zone: %s' %ID)
        
def geonotify_svc(ID='', ac_class='unknown', latitude=0.0, longitude=0.0, agl_altitude=0.0, velocity=0.0, control_lat=0.0, control_lon=0.0, control_alt=0.0, time_mark=0, emergency_status='normal_flight'):
    global tracks
    tracks[ID] = {'ac_class':ac_class, 'latitude': latitude, 'longitude': longitude, 'velocity': velocity, 'control_lat': control_lat, 'control_lon': control_lon, 'agl_altitude':agl_altitude, 'tlu':env.now, 'status':'active'}
#    print('received track update for ',ID)
    check_zones_track_intrusion(ID)

def receive_alert_svc(message=''):
    print('Alert received>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>',message,'<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')
    
# Service support functions

# Check all active zones against a single track
def check_zones_track_intrusion(track):
    for zone in notification_zones:
        check_zone_track_intrusion(zone, track)
        
# check a single zone and single track for intrusion
def check_zone_track_intrusion(zone, track):
    zone_dct = get_zone(zone)
    track_dct = get_track(track)
    if track_dct['tlu']+TRACK_TIMEOUT < env.now:
        alert_message('track %s stale, deleting' % track)
        tracks[track]['status'] = 'inactive'
        for zone_chk in notification_zones:
            if track in notification_zones[zone_chk]['inside']:
                notification_zones[zone_chk]['inside'].remove(track)
        return
#    print('checking zone for intrusion:',zone,track)
    if zone_dct['status'] == 'active':
        if inside(zone, track) and not track in zone_dct['inside'] and track_dct['ac_class'] in notify_lst:
#        print('intrusion alert',zone,track)
            zone_dct['inside'].append(track)
            alert(zone, track,'entered')
        elif track in zone_dct['inside'] and not inside(zone, track):
            zone_dct['inside'].remove(track)
            alert(zone, track,'departed')
            
# Simple alert message
def alert_message(msg):
    pub.sendMessage('alert', message=msg)

# Alert message customized to report zone and track
def alert(zone, track, msg):
    track_dct = get_track(track)
    zone_dct = get_zone(zone)
    ac_class = track_dct['ac_class']
    pub.sendMessage('alert.%s' % (zone_dct['ID']), message='A %s (%s) has %s notification zone: %s' %(ac_class,track,msg,zone_dct['ID']))

# return the dict related to zone
def get_zone(zone):
    if type(zone) == str:
        zone_dct = notification_zones[zone]
    else:
        zone_dct = zone
    return zone_dct

# return the dict related to a given track
def get_track(track):
    if type(track) == str:
        track_dct = tracks[track]
    else:
        track_dct = track
    return track_dct

# Return True if track is in a current zone or False if it is not
def inside(zone, track):
    zone_dct = get_zone(zone)
    track_dct = get_track(track)
    distance = latlon_distance((zone_dct['center'][0], zone_dct['center'][1]), (track_dct['latitude'], track_dct['longitude']))
#    print('distance from intrusion', geodesic_scalar(distance)-zone_dct['radius'])
    if distance < zone_dct['radius'] and track_dct['agl_altitude'] > 0.0: # TBD altitude is treated inconsistently
        return True
    return False

# Extract the scalar distance from the geodesic class. There is probably a cleaner way to do this.
def geodesic_scalar(geo):
    return float(str(geo).split(' ')[0])
    
# Manage changes to zone status
def update_zone_status(zone):
    if zone['status'] == 'active' and zone['end_time'] < env.now:
        alert_message('zone %s has expired' % zone['ID'])
        zone['status']='expired'
        zone['inside'] = []
    elif zone['status'] !=  'active' and zone['start_time'] <= env.now <= zone['end_time']:
        alert_message('zone %s is now active' % zone['ID'])
        zone['status']='active'

# periodically check for intrusions. It could be used to do dead reckoning which is not happening currently. It is also used to flag stale tracks and
# manage zone status
def check_intrusions():
    while True:
#        print('checking for dead reckon intrusions',env.now)
        for zone, zone_dct in notification_zones.items():
            update_zone_status(zone_dct)
            if zone_dct['status'] == 'active':
                for track in tracks:
                    if tracks[track]['status'] == 'active':
                        check_zone_track_intrusion(zone, track) 
        print('zones',notification_zones)
        yield env.timeout(NOTIFICATION_CHECK_INTERVAL)

# Compute distance between to lat/lon points
def latlon_distance(Pt1, Pt2):
    return geopy.distance.geodesic(Pt1, Pt2)

# The V0 geonotification service supports the following messages:

pub.subscribe(geonotify_svc, 'adsb')                        # Receive track information
pub.subscribe(receive_alert_svc, 'alert')                   # Generate an alert message to the user
pub.subscribe(delete_notification_zone, 'delete_zone')      # Allow a user to delete a zone
pub.subscribe(create_notification_zone, 'create_zone')      # Allow a user to create a zone



    
