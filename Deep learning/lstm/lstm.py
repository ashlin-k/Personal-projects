import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
from tensorflow.compat.v1 import layers
import numpy as np

import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from kanawaty_layers import *


class LSTM(tf.Module):
    
    def __init__(self):
        
        self.embedding_layer = tf.keras.layers.Embedding(num_distinct_words, embedding_output_dims, input_length=max_sequence_length)