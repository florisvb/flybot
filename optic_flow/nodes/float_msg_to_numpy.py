import numpy as np

def get_numpy_array(shape, data):
    data_array = np.asarray(data).reshape(shape)
    return data_array
    
