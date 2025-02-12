# GNSS
GNSS code for the autonomous Vehicle

This Library should also use another GitHub repository called RTKLIB by tomojitakasu [here](https://github.com/tomojitakasu/RTKLIB) for better accuracy.
## Steps to activate RTKLIB and GNSS.py:
1. go to terminal and type cd RTKLIB/app/str2str/gcc
2. Since this project is taking place in Ottawa, type in the terminal ./str2str -in ntrip://YOUR_EMAIL:none@rtk2go.com:2101/CanalTerris -out serial://ttyACM0:115200
3. Open a new terminal and then type python3 GNSS.py, depending on where GNSS.py is stored.
