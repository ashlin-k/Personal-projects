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
    
    def __call__(self, x : tf.Tensor) -> (tf.Tensor):

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