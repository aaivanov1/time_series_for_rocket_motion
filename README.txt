
Paper and source code for path planning app:
    https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/DynamicWindowApproach
    https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf


To run the code to generate the data for the moving objects:
    (1) change the code inside new_dynamic_window_approach.py, line 15 to say 'True'
    (2) run the following: 
            python new_dynamic_window_approach.py > moving_objects.csv
    (3) step 2 will generate the moving_objects.csv and png/dynamic_window_moving_objects.png
    (4) all code differences between the original and the modified file are in all-differences.png


To run the code to generate the data for the static objects:
    (1) change the code inside new_dynamic_window_approach.py, line 15 to say 'False'
    (2) run the following: 
            python new_dynamic_window_approach.py > static_objects.csv
    (3) step 2 will generate the static_objects.csv and png/dynamic_window_static_objects.png


To run the code to generate timeseries predictions using the Facebook prophet library:
    (1) pip install fbprophet
    (2) run the following:
            python prophet.py
    (3) step 2 will generate png/moving_objects_x0_Series.png, png/moving_objects_x1_Series.png
                         and png/static_objects_x0_Series.png, png/static_objects_x0_Series.png

To run jupyter notebook:
    jupyter notebook Prophet.ipynb

About fbprophet:
    https://facebook.github.io/prophet/
    https://facebook.github.io/prophet/docs/quick_start.html
    https://facebook.github.io/prophet/docs/quick_start.html#python-api
    https://facebook.github.io/prophet/docs/non-daily_data.html
    https://peerj.com/preprints/3190/


Unrelated but interesting:
    https://labs.consol.de/development/2018/10/31/introduction-to-timescale-db.html

