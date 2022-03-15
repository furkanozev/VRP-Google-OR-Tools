## HTTP Microservice with Google OR-tools

# In this project, I used some optional constrainst, that are;
	* "capacity": Initial carboy capacity of the vehicle.
	* "delivery": The amount of carboy that wil lbe delivered in this job.

# For creating Rest API, I used Flask framework in Python3.6
# For solving vehicle routing problem with constraints, I used Google OR-tools

# I used docker tool for microservice

	* To build: "docker build --tag route-app ."
	* To run: "docker run -d -p 5000:5000 route-app"

# You can request with input file (.json)
	* host: localhost (127.0.0.1)
	* port: 5000

# Furkan Özev