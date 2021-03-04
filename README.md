# proj2_dani_lerner

Libraries 

    import os
    import cv2
    import time
    import math
    import queue
    import operator
    import numpy as np

All libraries are standard python libraries (other than opencv). No pip3 install statements were used to generate the code. 
Please further note that the "main.py" python script imports the BFS.py script. Therefore, they must be kept in the same directory.

To run the code, change the directory to proj2_dani_lerner folder. i.e.

    cd your_path/proj2_dani_lerner

The name of the python file is simply "main.py". 
To run the main, type ...

    python main.py

... This will run all 3 test cases. Note that I added the last one to display the code's understanding when no possible path is detected.

To run a given test case, merely go into main.py and comment out everything but the given test case initialization and find_path function.

The search will prompt the user for start and goal positions. 
Please follow all terminal instructions once the program starts.