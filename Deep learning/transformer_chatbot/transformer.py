import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
from tensorflow.compat.v1 import layers
import numpy as np
from typing import Callable
from embedding import *

# tutorial: https://medium.com/tensorflow/a-transformer-chatbot-tutorial-with-tensorflow-2-0-88bf59e66fe2
# reference code: https://github.com/bryanlimy/tf2-transformer-chatbot/blob/master/transformer/model.py

def create_padding_mask(x):
    mask = tf.cast(tf.math.equal(x, 0), dtype=tf.float32)
    return mask[:, tf.newaxis, tf.newaxis, :]

def create_look_ahead_mask(x):
    seq_len = tf.shape(x)[1]
    look_ahead_mask = 1 - tf.linalg.band_part( \
        tf.ones((seq_len, seq_len), dtype=tf.float32), -1, 0)
    padding_mask = create_padding_mask(x)
    return tf.maximum(look_ahead_mask, padding_mask)

def scaled_dot_product_attention(query : tf.Tensor, key : tf.Tensor, value : tf.Tensor, \
    mask : tf.Tensor=None) -> (tf.Tensor):

    '''
    Computes the attention weights for each word, ensuring that we keep word of interest
    and discard irrelevant words.
    '''

    # attention = softmax,k((Q*K^T) / sqrt(dk)) * V
    # dk is is the depth, or number of columns, in K

    qk = tf.matmul(query, key, transpose_b=True)
    depth = tf.cast(key.shape[-1], tf.float32)
    logits = qk / tf.math.sqrt(depth)

    if mask is not None:
        logits += (mask * -1e9)
    
    attention_weights = tf.nn.softmax(logits, axis=-1)

    return tf.matmul(attention_weights, value)

class MultiHeadAttention(tf.Module):

    '''
    This class takes in a dictionary input {"query" : ..., "key" : ..., "value" : ...} and 
    runs these through Dense layers and then splits into multiple heads. The attention output
    for each head is then concatenated and run through a final Dense layer.
    Query, key, and value are split into multiple heads because it allows the model to jointly 
    attend to information at different positions from different representational spaces.  

    d_model = number of dimensions
    num_heads = number of heads
    '''

    def __init__(self, d_model : int, num_heads : int, name : str="multi_head_attention"):

        super(MultiHeadAttention, self).__init__(name=name)

        self.num_heads = num_heads
        self.d_model = d_model

        assert d_model % num_heads == 0

        self.depth = d_model

        self.query_dense = layers.Dense(units=d_model)
        self.key_dense = layers.Dense(units=d_model)
        self.value_dense = layers.Dense(units=d_model)
        self.final_dense = layers.Dense(units=d_model)

    def split_heads(self, inputs : tf.Tensor, batch_size : int) -> (tf.Tensor):

        inputs = tf.reshape(inputs, shape=(batch_size, -1, self.num_heads, self.depth))
        return tf.transpose(inputs, perm=[0,2,1,3])
    
    def call(self, inputs : dict) -> (tf.Tensor):

        query, key, value, mask = inputs['query'], inputs['key'], inputs['value'], inputs['mask']
        batch_size = query.shape[0]

        # linear layers
        query = self.query_dense(query)
        key = self.key_dense(key)
        value = self.value_dense(value)

        # split heads
        query = self.split_heads(query, batch_size)
        key = self.split_heads(key, batch_size)
        value = self.split_heads(value, batch_size)

        scaled_attn = scaled_dot_product_attention(query, key, value, mask)
        scaled_attn = tf.transpose(scaled_attn, perm=[0,2,1,3])

        concat_attn = tf.reshape(scaled_attn, shape=(batch_size, -1, self.d_model))

        output = self.final_dense(concat_attn)

        return output

class PositionalEncoding(tf.Module):

    '''
    Positional encoding is added to give the model some information about the relative 
    position of the words in the sentence. The positional encoding vector is added to the 
    embedding vector.
    '''

    def __init__(self, position : int, d_model : int):

        super(PositionalEncoding, self).__init__()
        self.pos_encoding = self.positional_encoding(position, d_model)

    def get_angles(self, position : tf.Tensor, i : tf.Tensor, d_model : int) -> (float):

        angles = 1 / tf.pow(10000, (2 * i//2) / tf.cast(d_model, tf.float32))  # // is integer division
        return position * angles
    
    def positional_encoding(self, position : int, d_model : int) -> (tf.Tensor):

        angle_rads = self.get_angles( \
            position=tf.range(position, dtype=tf.float32)[:, tf.newaxis],   # [:, tf.newaxis] makes it 2D col vector
            i=tf.range(d_model, dtype=tf.float32)[tf.newaxis, :],           # [tf.newaxis, :] makes it 2D row vector
            d_model=d_model)
        
        # apply sine to even indices in the array starting at 0
        sines = tf.math.sin(angle_rads[:, 0::2])

        # apply cosine to odd indices in the array starting at 1
        cosines = tf.math.cos(angle_rads[:, 1::2])

        pos_encoding = tf.concat([sines, cosines], axis=-1)[tf.newaxis, ...]

        return tf.cast(pos_encoding, tf.float32)
    
    def call(self, inputs : tf.Tensor) -> (tf.Tensor):

        return inputs + self.pos_encoding[:, :inputs.shape[1], :]

class InputLayer(tf.Module):

    def __init__(self):

        self.input = None
    
    def __call__(self, x : tf.Tensor) -> (tf.Tensor):

        self.input = x
        return x

class DropoutLayer(tf.Module):
    
    def __init__(self, dropout_rate : float=0.0, name="dropout_layer"):
        super().__init__(name=name)

        self.dropout = dropout_rate
    
    def __call__(self, x : tf.Tensor) -> (tf.Tensor):
        return tf.nn.dropout(x, self.dropout, seed=1)

class NormalizationLayer(tf.Module):

    def __init__(self, epsilon : float = 1e-6, axes : int=[1], name=None):
        super().__init__(name=name)

        self.epsilon = epsilon
        self.axes = axes

    def __call__(self, x : tf.Tensor) -> (tf.Tensor):

        mean_i = tf.reduce_sum(x, axis=self.axes) / x.shape[1]
        var_i = tf.reduce_sum((x - mean_i) ** 2) / x.shape[1]
        mean_i = tf.reshape(mean_i, shape=(mean_i.shape[0], -1, mean_i.shape[-1]))
        var_i = tf.reshape(var_i, shape=(var_i.shape[0], -1, var_i.shape[-1]))
        x_norm = (x - mean_i) / tf.math.sqrt(var_i + self.epsilon)

        return x_norm

class LambdaLayer(tf.Module):

    def __init__(self, expression : Callable[[tf.Tensor], tf.Tensor], name : str=None):

        super().__init__(name=name)
        self.expression = expression
    
    def __call__(self, x : tf.Tensor) -> (tf.Tensor):

        return self.expression(x)

class EncoderLayer(tf.Module):
    
    def __init__(units : int, d_model : int, num_heads : int, dropout : float, name : str="encoder_layer"):

        super().__init__(name=name)
        self.units = units
        self.d_model = d_model
        self.num_heads = num_heads
        self.dropout = dropout
    
    def __call__(self, x : tf.Tensor, padding_mask : tf.Tensor) -> (tf.Tensor):

        inputs = InputLayer(name="encoder_inputs")(x)
        padding_mask = InputLayer(name="padding_mask")(padding_mask)

        attention = MultiHeadAttention(self.d_model, self.num_heads, name="attention") \
            ({
                'query': inputs,
                'key': inputs,
                'value': inputs,
                'mask': padding_mask
            })

        attention = DropoutLayer(self.dropout)
        attention = NormalizationLayer(axes=[1])(inputs + attention)

        outputs = layers.Dense(units=self.units, activation='relu')(attention)
        outputs = layers.Dense(units=self.d_model)(outputs)
        outputs = DropoutLayer(dropout_rate=self.dropout)(outputs)
        outputs = NormalizationLayer(axes=[1])(attention + outputs)

        return outputs

class DecoderLayer(tf.Module):
    
    def __init__(self, units: int, d_model : int, num_heads : int, dropout : float, name="decoder_layer"):

        super().__init__(name=name)
        self.units = units
        self.d_model = d_model
        self.num_heads = num_heads
        self.dropout = dropout

    def __call__(self, x : tf.Tensor, encoder_outputs : tf.Tensor, padding_mask : tf.Tensor, \
        lookahead_mask : tf.Tensor) -> (tf.Tensor):

        inputs = InputLayer(name="inputs")(x)
        enc_outputs = InputLayer(name="encoder_outputs")(encoder_outputs)
        look_ahead_mask = InputLayer(name="look_ahead_mask")(lookahead_mask)
        padding_mask = InputLayer(name='padding_mask')(padding_mask)

        attention1 = MultiHeadAttention(d_model, num_heads, name="attention_1")\
            (inputs={
            'query': inputs,
            'key': inputs,
            'value': inputs,
            'mask': look_ahead_mask })

        attention1 = NormalizationLayer()(attention1 + inputs)

        attention2 = MultiHeadAttention(d_model, num_heads, name="attention_2")\
            (inputs={
            'query': attention1,
            'key': enc_outputs,
            'value': enc_outputs,
            'mask': padding_mask })
        attention2 = DropoutLayer(dropout_rate=dropout)(attention2)
        attention2 = NormalizationLayer()(attention2 + attention1)

        outputs = layers.Dense(units=units, activation='relu')(attention2)
        outputs = layers.Dense(units=d_model)(outputs)
        outputs = layers.Dropout(rate=dropout)(outputs)
        outputs = NormalizationLayer()(outputs + attention2)

        return outputs

class Transformer(tf.Module):

    def __init__(self, vocab_size : int, num_layers : int, units : int, d_model : int, \
        num_heads : int, dropout : float, name : str="transformer"):

        super().__init__(name=name)
        self.vocab_size = vocab_size
        self.num_layers = num_layers
        self.units = units
        self.d_model = d_model
        self.num_heads = num_heads
        self.dropout = dropout
    
    def __call__(self, x : tf.Tensor, dec_inputs : tf.Tensor) -> (tf.Tensor):

        inputs = InputLayer(name="inputs")(x)
        dec_inputs = InputLayer(name="dec_inputs")(dec_inputs)

        enc_padding_mask = LambdaLayer(create_padding_mask, name='enc_padding_mask')(inputs)

        # mask the future tokens for decoder inputs at the 1st attention block
        look_ahead_mask = LambdaLayer(create_look_ahead_mask, name='look_ahead_mask')(dec_inputs)

        # mask the encoder outputs for the 2nd attention block
        dec_padding_mask = LambdaLayer(create_padding_mask, name='dec_padding_mask')(inputs)

        enc_outputs = encoder(inputs, enc_padding_mask, self.vocab_size, self.num_layers, self.units, self.d_model, \
            self.num_heads, self.dropout)
        
        dec_outputs = decoder(dec_inputs, enc_outputs, look_ahead_mask, dec_padding_mask, \
            self.vocab_size, self.num_layers, self.units, self.d_model, self.num_heads, self.dropout)
        
        outputs = layers.Dense(units=self.vocab_size, name="outputs")(dec_outputs)

        return outputs

    def encoder(self, x : tf.Tensor, padding_mask : tf.Tensor) -> (tf.Tensor):

        inputs = InputLayer(name="inputs")(x)
        padding_mask = InputLayer(name="padding_mask")(padding_mask)

        embeddings = EmbeddingLayer(self.vocab_size, self.d_model)(inputs)
        embeddings *= tf.math.sqrt(tf.cast(self.d_model, tf.float32))
        embeddings = PositionalEncoding(self.vocab_size, self.d_model)(embeddings)

        outputs = DropoutLayer(dropout_rate=self.dropout)(embeddings)

        for i in range(num_layers):
            outputs = EncoderLayer(self.units, self.d_model, self.num_heads, \
                self.dropout, name="encoder_layer_{}".format(i),)(outputs, padding_mask)
        
        return outputs

    def decoder(self, x : tf.Tensor, encoder_outputs : tf.Tensor, padding_mask : tf.Tensor, \
        lookahead_mask : tf.Tensor) -> (tf.Tensor):

        inputs = InputLayer(name="inputs")(x)
        enc_outputs = InputLayer(name="encoder_outputs")(encoder_outputs)
        look_ahead_mask = InputLayer(name="look_ahead_mask")(lookahead_mask)
        padding_mask = InputLayer(name='padding_mask')(padding_mask)

        embeddings = EmbeddingLayer(self.vocab_size, self.d_model)(inputs)
        embeddings *= tf.math.sqrt(tf.cast(self.d_model, tf.float32))
        embeddings = PositionalEncoding(self.vocab_size, self.d_model)(embeddings)

        outputs = DropoutLayer(dropout_rate=self.dropout)(embeddings)

        for i in range(num_layers):
            outputs = DecoderLayer(self.units, self.d_model, self.num_heads, self.dropout, \
                name="decoder_layer_{}".format(i),)(x, encoder_outputs, padding_mask, lookahead_mask)
        
        return outputs

