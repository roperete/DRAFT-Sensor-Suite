# Grafana
Grafana is a WEB application that can consume varied sources of data and enables to create and configure rich dashboards. A free software version exists that we are going to use, see [open source grafana](https://grafana.com/oss/grafana/).

Using grafana enables to provide users with a feature rich graph visualisation, zooming over a period of time as well as enabling access to charts remotely.

## Install Grafana on the Raspberry Pi 5
to install the current version of grafana (arm 64) on raspi, directly, use the following commands:

```
sudo apt-get install -y adduser libfontconfig1 musl   
wget https://dl.grafana.com/oss/release/grafana-rpi_12.0.2_armhf.deb  
sudo dpkg -i grafana-rpi_12.0.2_armhf.deb
```


**Remark** alternatively, it is possible to enable installing through the raspberry pi OS package manager, see [grafana on raspberry pi 5](https://grafana.com/tutorials/install-grafana-on-raspberry-pi/)
Note that you might need to tweak a bit some instructions that are obsolete, in particular about the keyring management: they are now stored in `/usr/share/keyrings` and not `/etc/apt/keyrings`.

We select the direct method.


### Starting Grafana
1. starting it once :   
	```
	sudo /bin/systemctl start grafana-server
	```	
2. starting it automatically, relying on systemd  
	```
	sudo /bin/systemctl daemon-reload
	sudo /bin/systemctl enable grafana-server
	```


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
- log in as admin
- go to administration/plugins and data
- search for Infinity
- install infinity
<img width="864" height="563" alt="Capture d’écran du 2025-07-15 15-42-28" src="https://github.com/user-attachments/assets/61a3c8ba-3574-436d-a4f3-addc28e5d986" />


**TODO** *decide where to put the CSV file*

#### Install Ininity Plugin




### Dashboards and visualisation

#### Creating a dashboard

#### Importing a predefined dashboard 

## A mini http server
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
