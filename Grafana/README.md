# Grafana charting
Using grafana enables to provide users with a feature rich graph visualisation, zooming over a period of time as well as enabling access to charts remotely.

## Install Grafana on the Raspberry pi

### Required plugin for CSV files
Install the Infinity plugin

*HOW TO*

### A mini http server
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


## Configuring Grafana

### Accounts

### Data source


### Dashboards and visualisation

#### Importing a predefined dashboard 