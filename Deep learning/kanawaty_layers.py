import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
from tensorflow.compat.v1 import layers
import numpy as np
from typing import Callable

# Generic custom layers that can be used for different deep learning algorithms

class InputLayer(tf.Module):
    
    def __init__(self, name : str="input_layer"):

        super(InputLayer, self).__init__(name=name)
    
    def __call__(self, x : tf.Tensor, shape : list=None) -> (tf.Tensor):

        if x.dtype != tf.float32:
            x = tf.cast(x, tf.float32)

        if shape is not None and list(x.shape) != shape:
            size_diff = tf.abs(tf.subtract(x.shape, shape))
            padding_size = tuple([x.shape[i] if size_diff[i] == 0 else size_diff[i].numpy() \
                for i in range(len(size_diff))])
            axis = np.argmax(size_diff)
            padding = tf.zeros(padding_size, dtype=tf.float32)
            
            return tf.concat((x, padding), axis=axis)

        return x

class DropoutLayer(tf.Module):
    
    def __init__(self, dropout_rate : float=0.0, name="dropout_layer"):
        super(DropoutLayer, self).__init__(name=name)

        self.dropout = dropout_rate
    
    def __call__(self, x : tf.Tensor, training : bool=True) -> (tf.Tensor):

        if training:
            return tf.nn.dropout(x, self.dropout, seed=1)
        else:
            return x

class LambdaLayer(tf.Module):
    
    def __init__(self, expression : Callable[[tf.Tensor], tf.Tensor], name : str="lambda_layer"):

        super(LambdaLayer, self).__init__(name=name)
        self.expression = expression
    
    def __call__(self, x : tf.Tensor) -> (tf.Tensor):

        return self.expression(x)