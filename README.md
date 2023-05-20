# mercator-origins-lemon

The Lemon M5 Stick sits in the GPS float box and is the virtual umbilical from the Diver to/from the interwebs via WiFi (a TP-Link MiFi hotspot or mobile phone with hotspot can be placed in the float box too). It also has responsibility for filtering out the tonne of GPS NMEA ASCII messages from 10 or so a second down to 2 per second - ie just the location messages - before sending down to the diver console (the Mako M5 Stick). It then listens for the binary-encoded telemetry messages sent up from Diver Console (the Mako M5 Stick), munges these into a JSON message containing metrics from both Mako and Lemon (75 metrics in total) and whizzes this off to the MQTT endpoint at Qubitro for data collection, map tracking and dashboard population.


