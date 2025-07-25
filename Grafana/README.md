# Grafana
Grafana is a WEB application that can consume varied sources of data and enables to create and configure rich dashboards. A free software version exists that we are going to use, see [open source grafana](https://grafana.com/oss/grafana/).

Using grafana enables to provide users with a feature rich graph visualisation, zooming over a period of time as well as enabling access to charts remotely.

## Install Grafana on the Raspberry Pi 5
to install the current version of grafana (arm 64) on raspi, using the package manager, use the following commands:

```
wget -q -O grafana.gpg.key https://apt.grafana.com/gpg.key
sudo gpg  -o /etc/apt/keyrings/grafana.gpg --dearmor grafana.gpg.key
echo "deb [signed-by=/etc/apt/keyrings/grafana.gpg] https://apt.grafana.com stable main" | sudo tee /etc/apt/sources.list.d/grafana.list

```
to

- get the signature key of grafana and store it appropriately
- add the grafana repository in the apt source list

From there, you can install grafana with the pack	ge manager on the raspberry pi.

**NOTE** : the default install did not work, the nightly built as of 15/Jul/2025 did.   
The installed package is the open source 'gafana-nightly-12.1.0-253421' 64 bits.



### Starting Grafana
Since installed from a package, it is configured to start automatically?

- it can be stopped with    
	```
	sudo /bin/systemctl stop grafana-server
	```
- restarted with  
	```
	sudo /bin/systemctl start grafana-server
	```	
- prevented from automatic start with  
	```
	sudo /bin/systemctl disable grafana-server
	```
- reallowed with  
	```
	sudo /bin/systemctl enable grafana-server
	```

Look for `systemctl` manual for more details.

## Configuring Grafana
Once installed and started, 
- if your raspberry pi is connected to a network with IP address `IPADDR`, then you can connect from a bowser on a machine that has access to the same nework, with URL: `http://<IPADDR>:3000`
- if your raspberry pi is not connected to a network, then use the raspberry browser and connect to `http://127.0.0.1:3000` or `http://localhost:3000`


### Accounts
At start, you have to choose a password for the admin account.

Once dashboards created, create a `user` account that can read dashboards but not edit them.
This acount will be used by generic users reading the graphs.

### Data source
Create a data source for the csv file. Reading CSV files rely on the *Infinity* plugin.

#### Install Ininity Plugin
- log in as admin
- go to administration/plugins and data
- search for Infinity
- install infinity

<img width="432" height="282" alt="Capture d’écran du 2025-07-15 15-42-28" src="https://github.com/user-attachments/assets/1e879941-46be-4c18-8b9a-0b58cfc4c4c0" />


<img width="459" height="162" alt="Install2" src="https://github.com/user-attachments/assets/e0ca5327-6bc5-4505-82d0-00d0d9a96122" />


#### Http Server 
Install and configure a small HTTP server that will serve the data file (Data/sensor_data.csv)to the Infinity Data Source.

- Install `mini-httpd` using the raspberry pi pckage manager, the package is named Small HTTP server , mini-httpd.
- To start it, use  
	```
	mini_httpd -p 3003 -d /home/spring/Data
	```
	
	or the shell script `start_http.sh`
	
It will use the port 3003 and serve only files from the folder `/home/spring/Data`

#### Configure Data Source

- Go to Connections/DataSources;
- select Add data source
- choose type "Infinity"
- Set its name (Arduino0)
- Set Authentication (No Auth)
- Set URL
<img width="494" height="262" alt="URL" src="https://github.com/user-attachments/assets/03ce0fb3-fcc5-4fe4-bd53-ed6aeae71af3" />


- Hit Save & Test, that should provide a positive return

<img width="386" height="212" alt="save-test" src="https://github.com/user-attachments/assets/d544dc1a-35d2-457a-8678-868e54a976b8" />



### Dashboards and visualisation
#### Importing a predefined dashboard 
A dashboard export named `sgp30-MQ.json` is ready for import. It contain gases measurements.

**TODO** put images

- Go to dashboards
- select Add new dashboard
- select Import
    - select the json file to upload
- save and test
   
#### Creating a dashboard

## Http server discussion
The infinity plugin requires a way to GET the file from a HTTP server. The server needs a minimal set of features

* support GET calls
* provide a minimal security
     * limit access to the directory that contains the file(s) to serve -- ideally enable to chroot to that folder
     * optionally support https
* enable to select a port to server else than 80
* have a light memory and CPU footprint (for example APACHE is overkill for that purpose)
* start at boot

**Possible choices**

* mini_httpd
    * very light
    * requires scripting to start at boot
* lighttpd
    * light
    * has a standard configuration file
    * starts as a daemon under systemd

**Final choice**
use mini_httpd a it is lighter and the drawbacks are not blocking.

# Start it All
- grafana is set to start at boot
- go to `/home/spring/DRAFT-Sensor-Suite`
- run `sh start_http.sh`
- run  `python3 SensorReading.py &`

Note:
to view the collected CSV; two alternatives
1. connect with a browser to the raspbery pi on port 3003   
 example `http://localhost:3003`
2. on a raspberry pi terminal or ssh terminal :  
`tail -f ~spring/Data/sensor_data.csv`   
End this display with Ctl-C.


