Complete the following steps to compile the software 


a) PREVIOUS: 

   The following packages are required: Python, gcc 

   These packages can be installed in UBUNTU as follows:

   - Installing Python (this is necessary for gLAB GUI and for graph.py):

     sudo apt-get install python python-wxtools python-matplotlib python-tk python-wxgtk2.8 python-mpltoolkits.basemap python-mpltoolkits.basemap-data python-numpy

   - Additional packages for Python3 (in case user wants to execute graph.py in Python3):

     sudo apt-get install python3-matplotlib python3-tk python3-mpltoolkits.basemap python-mpltoolkits.basemap-data python3-numpy

   - Installing C compiler (gcc)

     sudo apt-get install gcc


b) Compile the program by executing:
     
      make

c) Executing gLAB

   - Graphic user Interface

     ./gLAB_GUI.py


   - Command line mode

     ./gLAB_linux


   - Plotting module

     ./graph.py
