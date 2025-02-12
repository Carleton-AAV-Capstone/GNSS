# GNSS
GNSS code for the autonomous Vehicle

This Library should also use another GitHub repository called RTKLIB by tomojitakasu [here](https://github.com/tomojitakasu/RTKLIB) for better accuracy.
## Steps to activate RTKLIB and GNSS.py:

### INSTALL RTKLIB (Suggested section)
- clone rtklib ```git clone https://github.com/tomojitakasu/RTKLIB && cd RTKLIB```
- build the str2str program ```cd app/str2str/gcc/ && make ```


### Run str2str
1. go to terminal and type cd RTKLIB/app/str2str/gcc
2. Since this project is taking place in Ottawa, type in the terminal ```./str2str -in ntrip://<YOUR_EMAIL>:none@rtk2go.com:2101/CanalTerris -out serial://<USB PORT>``` e.g. :```./str2str -in ntrip://YOUR_EMAIL:none@rtk2go.com:2101/CanalTerris -out serial://ttyACM0:115200```
4. Open a new terminal and then type python3 GNSS.py, depending on where GNSS.py is stored.


## OR use docker

1.  save the usb port for the reciever in an environment variable ```export USB_PORT=ttyACM0```
2.  clone the repo ```git clone https://github.com/Carleton-AAV-Capstone/GNSS && cd GNSS```
3. build the container ```docker build -t <name of the container> .```
4. run the container ```sudo docker run -it --device=/dev/ttyACM0 --privileged -e EMAIL_ADDR=<your email> -e USB_PORT=${USB_PORT} <name of the container>```
