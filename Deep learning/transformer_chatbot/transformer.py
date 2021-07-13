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

# tutorial: https://medium.com/tensorflow/a-transformer-chatbot-tutorial-with-tensorflow-2-0-88bf59e66fe2
#           https://www.tensorflow.org/text/tutorials/transformer 
# reference code: https://github.com/bryanlimy/tf2-transformer-chatbot/blob/master/transformer/model.py

def broadcast_padding_mask(m : tf.Tensor, n_channels : int, n_heads : int) -> (tf.Tensor):
    m2 = tf.transpose(m, perm=[0,1,3,2])        # 64,1,40,1
    m2 = tf.broadcast_to(m2, (m2.shape[0], m2.shape[1], m2.shape[2], n_channels))   # 64,1,40,5
    m2 = tf.reshape(m2, (m2.shape[0], -1, n_heads, n_channels)) # 64,5,8,5
    m2 = tf.transpose(m2, perm=[0, 2, 1, 3])  # 64,8,5,5
    return m2

def broadcast_lookahead_mask(m : tf.Tensor, n_channels : int, n_heads : int) -> (tf.Tensor):
    m2 = tf.reshape(m, (m.shape[0], n_heads, n_channels, -1))
    # m2 = tf.transpose(m2, perm=[0, 2, 1, 3])
    m2 = tf.matmul(m2, m2, transpose_b=True)
    m2 = tf.where(tf.greater(m2, 0), 1, 0);
    return m2

def create_padding_mask(x : tf.Tensor) -> (tf.Tensor):
    mask = tf.cast(tf.math.equal(x, 0), dtype=tf.float32)
    return mask [:, tf.newaxis, tf.newaxis, :]

def create_look_ahead_mask(x : tf.Tensor) -> (tf.Tensor):
    seq_len = tf.shape(x)[1]
    look_ahead_mask = 1 - tf.linalg.band_part(tf.ones((seq_len, seq_len), dtype=tf.float32), -1, 0)
    padding_mask = create_padding_mask(x)
    return tf.maximum(look_ahead_mask, padding_mask)
    # return look_ahead_mask

def scaled_dot_product_attention(query : tf.Tensor, key : tf.Tensor, value : tf.Tensor, \
    padding_mask : tf.Tensor=None, lookahead_mask : tf.Tensor=None) -> (tf.Tensor):

    '''
    Computes the attention weights for each word, ensuring that we keep word of interest
    and discard irrelevant words.
    Queries is a set of vectors you want to calculate attention for.
    Keys is a set of vectors you want to calculate attention against.
    Values equal keys.
    1. We dot product the query with each key as a measure of similarity.This produces 
    weights. 
    2. The weights are then put through a softmax function to get a value
    between 0 and 1. This becomes important to get a "weighted-average" of the value vectors.
    3. We then multiply the weights times the values to get our final single output word vector 
    representation.
    '''

    # attention = softmax,k((Q*K^T) / sqrt(dk)) * V
    # dk is is the depth, or number of columns, in K

    qk = tf.matmul(query, key, transpose_b=True)    # (..., seq_len_q, seq_len_k)
    depth = tf.cast(key.shape[-1], tf.float32)
    logits = qk / tf.math.sqrt(depth)

    if padding_mask is not None:
        n_heads = qk.shape[1]
        n_channels = qk.shape[-1]
        m = broadcast_padding_mask(padding_mask, n_channels, n_heads)
        logits += (m * -1e9)
    
    if lookahead_mask is not None:
        n_heads = qk.shape[1]
        n_channels = qk.shape[-1]
        m = broadcast_lookahead_mask(lookahead_mask, n_channels, n_heads)
        logits += (m * -1e9)
    
    # softmax is normalized on the last axis (seq_len_k) so that the scores
    # add up to 1.
    attention_weights = tf.nn.softmax(logits, axis=-1)  # (..., seq_len_q, seq_len_k)
    output = tf.matmul(attention_weights, value)  # (..., seq_len_q, depth_v)

    return output 

class MultiHeadAttention(tf.Module):

    '''
    This class takes in a dictionary input {"query" : ..., "key" : ..., "value" : ...}.
    Query, key, and value are split into multiple heads because it allows the model to jointly 
    attend to information at different positions from different representational spaces.  

    from: https://stats.stackexchange.com/questions/421935/what-exactly-are-keys-queries-and-values-in-attention-mechanisms

    "There are two self-attending (xN times each) blocks, separately for inputs and outputs 
    plus cross-attending block transmitting knowledge from inputs to outputs.

    Each self-attending block gets just one set of vectors (embeddings added to positional values). 
    In this case you are calculating attention for vectors against each other. So Q=K=V. You just 
    need to calculate attention for each q in Q.

    Cross-attending block transmits knowledge from inputs to outputs. In this case you get K=V 
    from inputs and Q are received from outputs. I think it's pretty logical: you have database of 
    knowledge you derive from the inputs and by asking Queries from the output you extract required 
    knowledge."

    d_model = number of dimensions
    num_heads = number of heads
    '''

    def __init__(self, d_model : int, num_heads : int, name : str="multi_head_attention"):

        super(MultiHeadAttention, self).__init__(name=name)

        self.n_heads_encoder = num_heads
        self.d_model = d_model

        assert d_model % num_heads == 0

        self.depth = d_model

        self.query_dense = layers.Dense(units=d_model)
        self.key_dense = layers.Dense(units=d_model)
        self.value_dense = layers.Dense(units=d_model)
        self.final_dense = layers.Dense(units=d_model)

    def split_heads(self, inputs : tf.Tensor, batch_size : int) -> (tf.Tensor):

        inputs = tf.reshape(inputs, (batch_size, -1, self.n_heads_encoder, self.depth))
        return tf.transpose(inputs, perm=[0, 2, 1, 3])
    
    def __call__(self, inputs : dict) -> (tf.Tensor):

        query, key, value, padding_mask, lookahead_mask = \
            inputs['query'], inputs['key'], inputs['value'], inputs['padding_mask'], inputs['lookahead_mask']
        batch_size = query.shape[0]

        # linear layers
        query = self.query_dense(query)     # (batch_size, seq_len, d_model)
        key = self.key_dense(key)
        value = self.value_dense(value)

        # split heads
        # Instead of one single attention head, Q, K, and V are split into multiple heads 
        # because it allows the model to jointly attend to information from different 
        # representation subspaces at different positions. After the split each head has a 
        # reduced dimensionality, so the total computation cost is the same as a single head 
        # attention with full dimensionality.
        query = self.split_heads(query, batch_size)     # (batch_size, num_heads, seq_len_q, depth)
        key = self.split_heads(key, batch_size)
        value = self.split_heads(value, batch_size)

        scaled_attn = scaled_dot_product_attention(query, key, value, padding_mask=padding_mask, \
            lookahead_mask=lookahead_mask)
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

    def __init__(self, position : int, d_model : int, name : str="encoding_layer"):

        super(PositionalEncoding, self).__init__(name=name)
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
    
    def __call__(self, inputs : tf.Tensor) -> (tf.Tensor):

        return inputs + self.pos_encoding[:, :inputs.shape[1], :]

class NormalizationLayer(tf.Module):

    def __init__(self, epsilon : float = 1e-6, samples_axis : int=1, name : str="normalization_layer"):
        super(NormalizationLayer, self).__init__(name=name)

        self.epsilon = epsilon
        self.samples_axis = samples_axis

    def __call__(self, x : tf.Tensor) -> (tf.Tensor):

        numSamples = x.shape[self.samples_axis]
        mean_var_shape = self.get_shape(x.shape)
        
        mean_i = tf.reduce_sum(x, axis=self.samples_axis) / numSamples
        mean_i = tf.reshape(mean_i, shape=mean_var_shape)
        var_i = tf.reduce_sum((x - mean_i) ** 2, axis=self.samples_axis) / numSamples
        var_i = tf.reshape(var_i, shape=mean_var_shape)
        x_norm = (x - mean_i) / tf.math.sqrt(var_i + self.epsilon)

        return x_norm
    
    def get_shape(self, input_dim : tuple) -> (tuple):
        ret_shape = [i for i in input_dim]
        ret_shape[self.samples_axis] = -1
        return ret_shape

class EncoderLayer(tf.Module):
    
    def __init__(self, units : int, d_model : int, num_heads : int, dropout : float, name : str="encoder_layer"):

        super(EncoderLayer, self).__init__(name=name)
        self.units = units
        self.d_model = d_model
        self.n_heads_encoder = num_heads
        self.dropout = dropout

        self.input_layer = InputLayer(name="encoder_inputs")
        self.padding_input_layer = InputLayer(name="padding_mask")
        self.attn_layer = MultiHeadAttention(self.d_model, self.n_heads_encoder, name="attention")
        self.dropout_layer1 = DropoutLayer(dropout_rate=self.dropout, name="dropout1")
        self.norm_layer1 = NormalizationLayer(samples_axis=1, name="norm1")
        self.dense_output_layer1 = layers.Dense(units=self.units, activation='relu', name="dense1")
        self.dense_output_layer2 = layers.Dense(units=self.d_model, name="dense2")
        self.dropout_layer2 = DropoutLayer(dropout_rate=self.dropout, name="dropout2")
        self.norm_layer2 = NormalizationLayer(samples_axis=1, name="norm2")
    
    def __call__(self, x : tf.Tensor, padding_mask : tf.Tensor, training : bool=True) -> (tf.Tensor):

        inputs = self.input_layer(x)
        padding_mask = self.padding_input_layer(padding_mask)

        attention = self.attn_layer({
                'query': inputs,
                'key': inputs,
                'value': inputs,
                'padding_mask': padding_mask,
                'lookahead_mask' : None
            })

        attention = self.dropout_layer1(attention, training=training)
        attention = self.norm_layer1(inputs + attention)

        outputs = self.dense_output_layer1(attention)
        outputs = self.dense_output_layer2(outputs)
        outputs = self.dropout_layer2(outputs, training=training)
        outputs = self.norm_layer2(attention + outputs)

        return outputs

class DecoderLayer(tf.Module):
    
    def __init__(self, units: int, d_model : int, num_heads : int, dropout : float, name="decoder_layer"):

        super().__init__(name=name)
        self.units = units
        self.d_model = d_model
        self.n_heads_encoder = num_heads
        self.dropout = dropout

        self.input_layer = InputLayer(name="decoder_inputs")
        self.enc_out_input_layer = InputLayer(name="encoder_outputs")
        self.lookahead_mask_input_layer = InputLayer(name="lookahead_mask")
        self.padding_mask_input_layer = InputLayer(name="padding_mask")
        self.attn_layer1 = MultiHeadAttention(self.d_model, self.n_heads_encoder, name="attention1")
        self.norm_layer1 = NormalizationLayer(samples_axis=1, name="norm1")
        self.attn_layer2 = MultiHeadAttention(self.d_model, self.n_heads_encoder, name="attention2")
        self.dropout_layer1 = DropoutLayer(dropout_rate=self.dropout, name="dropout1")        
        self.norm_layer2 = NormalizationLayer(samples_axis=1, name="norm2")
        self.dense_output_layer1 = layers.Dense(units=self.units, activation='relu', name="dense1")
        self.dense_output_layer2 = layers.Dense(units=self.d_model, name="dense2")
        self.dropout_layer2 = DropoutLayer(dropout_rate=self.dropout, name="dropout2")
        self.norm_layer3 = NormalizationLayer(samples_axis=1, name="norm3")

    def __call__(self, x : tf.Tensor, encoder_outputs : tf.Tensor, padding_mask : tf.Tensor, \
        lookahead_mask : tf.Tensor, training : bool=True) -> (tf.Tensor):

        inputs = self.input_layer(x)
        enc_outputs = self.enc_out_input_layer(encoder_outputs)
        look_ahead_mask = self.lookahead_mask_input_layer(lookahead_mask)
        padding_mask = self.padding_mask_input_layer(padding_mask)

        attention1 = self.attn_layer1(inputs={
            'query': inputs,
            'key': inputs,
            'value': inputs,
            'padding_mask' : None,
            'lookahead_mask': look_ahead_mask })

        attention1 = self.norm_layer1(attention1 + inputs)

        attention2 = self.attn_layer2(inputs={
            'query': attention1,
            'key': enc_outputs,
            'value': enc_outputs,
            'padding_mask': padding_mask,
            'lookahead_mask' : None })
        attention2 = self.dropout_layer1(attention2, training=training)
        attention2 = self.norm_layer2(attention2 + attention1)

        outputs = self.dense_output_layer1(attention2)
        outputs = self.dense_output_layer2(outputs)
        outputs = self.dropout_layer2(outputs, training=training)
        outputs = self.norm_layer3(outputs + attention2)

        return outputs

class Transformer(tf.Module):

    def __init__(self, vocab_size : int, num_layers : int, units : int, d_model : int, \
        num_enc_heads : int, num_dec_heads : int, dropout : float, name : str="transformer"):

        super(Transformer, self).__init__(name=name)
        self.vocab_size = vocab_size
        self.num_layers = num_layers
        self.units = units
        self.d_model = d_model
        self.n_heads_encoder = num_enc_heads
        self.n_heads_decoder = num_dec_heads
        self.dropout = dropout

        # transformer layers
        self.input_layer = InputLayer(name="decoder_inputs")
        self.dec_input_layer = InputLayer(name="encoder_outputs")
        self.enc_padding_mask_layer = LambdaLayer(create_padding_mask, name='enc_padding_mask')
        self.lookahead_mask_layer = LambdaLayer(create_look_ahead_mask, name='look_ahead_mask')
        self.dec_padding_mask_layer = LambdaLayer(create_padding_mask, name='dec_padding_mask')
        self.dense_output_layer = layers.Dense(units=self.vocab_size, name="outputs")

        # encoder layers
        self.encoder_input_layer = InputLayer(name="encoder_inputs")
        self.encoder_padding_mask_input_layer = InputLayer(name="encoder_padding_mask")
        self.encoder_embedding_layer = tf.keras.layers.Embedding(self.vocab_size, self.d_model, name="encoder_embedding")
        self.encoder_pos_enc_layer = PositionalEncoding(self.vocab_size, self.d_model, name="encoder_pos_enc")
        self.encoder_dropout_layer = DropoutLayer(dropout_rate=self.dropout, name="encoder_dropout")
        self.encoder_enc_layers = [EncoderLayer(self.units, self.d_model, self.n_heads_encoder, \
                self.dropout, name="encoder_layer_{}".format(i),) for i in range(self.num_layers)]

        # decoder layers
        self.decoder_input_layer = InputLayer(name="decoder_inputs")
        self.decoder_enc_out_input_layer = InputLayer(name="decoder_enc_outputs")
        self.decoder_lookahead_mask_input_layer = InputLayer(name="decoder_lookahead_mask")
        self.decoder_padding_mask_input_layer = InputLayer(name="decoder_padding_mask")
        self.decoder_embedding_layer = tf.keras.layers.Embedding(self.vocab_size, self.d_model, name="decoder_embedding")
        self.decoder_pos_enc_layer = PositionalEncoding(self.vocab_size, self.d_model, name="decoder_pos_enc")
        self.decoder_dropout_layer = DropoutLayer(dropout_rate=self.dropout, name="decoder_dropout")
        self.decoder_dec_layers = [DecoderLayer(self.units, self.d_model, self.n_heads_decoder, \
                self.dropout, name="decoder_layer_{}".format(i),) for i in range(self.num_layers)]
    
    def __call__(self, inputs : tf.Tensor, dec_inputs : tf.Tensor, training : bool=True) -> (tf.Tensor):

        inputs = self.input_layer(inputs)
        dec_inputs = self.dec_input_layer(dec_inputs)

        # Encoder padding mask
        enc_padding_mask = self.enc_padding_mask_layer(inputs)

        # Used in the 1st attention block in the decoder.
        # It is used to pad and mask future tokens in the input received by
        # the decoder
        look_ahead_mask = self.lookahead_mask_layer(dec_inputs)

        # Used in the 2nd attention block in the decoder.
        # This padding mask is used to mask the encoder outputs.
        dec_padding_mask = self.dec_padding_mask_layer(inputs)

        enc_outputs = self.encoder(inputs, enc_padding_mask, training)
        
        dec_outputs = self.decoder(dec_inputs, enc_outputs, dec_padding_mask, look_ahead_mask, training)
        
        outputs = self.dense_output_layer(dec_outputs)

        return outputs

    def encoder(self, inputs : tf.Tensor, padding_mask : tf.Tensor, training : bool=True) -> (tf.Tensor):

        inputs = self.encoder_input_layer(inputs)
        padding_mask = self.encoder_padding_mask_input_layer(padding_mask)
        # padding_mask = tf.keras.Input(shape=(1, 1, None), name="padding_mask")

        embeddings = self.encoder_embedding_layer(inputs)
        embeddings *= tf.math.sqrt(tf.cast(self.d_model, tf.float32))
        embeddings = self.encoder_pos_enc_layer(embeddings)

        outputs = self.encoder_dropout_layer(embeddings, training=training)

        for i in range(self.num_layers):
            outputs = self.encoder_enc_layers[i](outputs, padding_mask, training)
        
        return outputs

    def decoder(self, inputs : tf.Tensor, encoder_outputs : tf.Tensor, padding_mask : tf.Tensor, \
        lookahead_mask : tf.Tensor, training : bool=True) -> (tf.Tensor):

        inputs = self.decoder_input_layer(inputs)
        enc_outputs = self.decoder_enc_out_input_layer(encoder_outputs)
        look_ahead_mask = self.decoder_lookahead_mask_input_layer(lookahead_mask)
        padding_mask = self.decoder_padding_mask_input_layer(padding_mask)

        embeddings = self.decoder_embedding_layer(inputs)
        embeddings *= tf.math.sqrt(tf.cast(self.d_model, tf.float32))
        embeddings = self.decoder_pos_enc_layer(embeddings)

        outputs = self.decoder_dropout_layer(embeddings, training=training)

        for i in range(self.num_layers):
            outputs = self.decoder_dec_layers[i](outputs, encoder_outputs, padding_mask, lookahead_mask, training)
        
        return outputs

