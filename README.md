# geonotify

This project provides a simulation context to allow the geonotification service to be exercised and tested. The simulation combined randomly generated uav's in a given area as well as integrates an ADSB feed via a commercial API. Two scenario files are available: test_scenario.json which defines the scenario and uav_data.json which defines performance data for various uav models.

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
