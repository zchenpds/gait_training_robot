import os
import shutil

def rectify_bag_names(path, trial_id=None):
    files = os.listdir(path)
    if trial_id:
        files = [f for f in files if f[4:7]==trial_id]
        if not files:
            print('Cannot find data' + trial_id + ' in ' + path)
    for file in files:
        name_old = os.path.join(path, file)
        name_new = os.path.join(path, file[:7] + '.bag')
        shutil.move(name_old, name_new)