import os
import pickle
from tkinter import filedialog as fd

from Models import MapModel


# Responsible for saving/loading a road_segments dict to/from a json file
def save_to_file(map_model: MapModel):
    filename = fd.asksaveasfilename(initialdir=os.path.normpath(os.getcwd() + os.sep + os.pardir)+"/paths",
                                    defaultextension=".pickle",
                                    filetypes=(("Pickle file", "*.pickle"), ("All files", "*.*")),
                                    title="Save path as...")
    outfile = open(filename, 'wb')

    pickle.dump(map_model, outfile)
    outfile.close()


# load from save file
def load_from_file() -> MapModel:
    filename = fd.askopenfilename(initialdir=os.path.normpath(os.getcwd() + os.sep + os.pardir)+"/paths",
                                  title="Select file")
    return load_from_filename(filename)


def load_from_filename(filename: str) -> MapModel:
    infile = open(filename, 'rb')
    map_model: MapModel = pickle.load(infile)

    # Warning message for version mismatch
    if map_model.instance_version != MapModel.version:
        print("\033[1m"f'WARNING: version mismatch between MapModel class version ({MapModel.version}) '
              f'and serialized version ({map_model.instance_version})'"\033[0m")

    infile.close()
    return map_model
