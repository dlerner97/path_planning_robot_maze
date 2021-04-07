# proj2_dani_lerner

## Dependencies

The following packages must be installed to run this code:

1. os               (std lib)
2. cv2
3. time             (std lib)
4. math             (std lib)
5. queue            (std lib)
6. heapq            (std lib)
7. operator         (std lib)
8. numpy as np      (std lib)

All libraries are standard python libraries (other than opencv).  
Please further note that the "main.py" python script imports the other scripts. Therefore, they must be kept in the same directory.

To run the code, change the directory to proj2_dani_lerner folder. i.e.

```bash
cd your_path/proj2_dani_lerner
```

To run a given test case, merely go into main.py and comment out everything but the given test case initialization and find_path function. To test discrete cases, comment out the

```python
action_set = search.Search.gen_robot_action_set()
```

line. This line resets the action set to a non-discrete case.

The name of the python file is simply "main.py".
To run the main, type ...

```bash
python main.py
```

The search will prompt the user for certain attributes and others can be defined in the main function.

Please follow all terminal instructions once the program starts.
