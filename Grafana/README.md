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
The isntalled package is the open source 'gafana-nightly-12.1.0-253421' 64 bits.



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

![Install](Grafana/Capture d’écran du 2025-07-15 15-42-28.png)

![Install 2nd step](Grafana/Install2.png)


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


### Dashboards and visualisation

#### Creating a dashboard

#### Importing a predefined dashboard 

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
